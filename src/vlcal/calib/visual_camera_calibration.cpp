#include <vlcal/calib/visual_camera_calibration.hpp>

#include <algorithm>
#include <limits>

#include <opencv2/imgproc.hpp>

#include <ceres/ceres.h>
#include <ceres/autodiff_first_order_function.h>

#include <sophus/se3.hpp>
#include <sophus/ceres_manifold.hpp>

#include <dfo/nelder_mead.hpp>

#include <vlcal/costs/nid_cost.hpp>
#include <vlcal/calib/view_culling.hpp>
#include <vlcal/calib/cost_calculator_nid.hpp>

namespace vlcal {

namespace {

Eigen::Isometry3d expmap_delta(const Eigen::Matrix<double, 6, 1>& xi) {
  Sophus::SE3d delta = Sophus::SE3d::exp(xi);
  return Eigen::Isometry3d(delta.matrix());
}

}  // namespace

VisualCameraCalibration::VisualCameraCalibration(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<VisualLiDARData::ConstPtr>& dataset,
  const VisualCameraCalibrationParams& params)
: params(params),
  proj(proj),
  dataset(dataset) {}

cv::Mat VisualCameraCalibration::to_grayscale(const cv::Mat& image) {
  if (image.channels() == 1) {
    return image;
  }
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  return gray;
}

CalibrationResult VisualCameraCalibration::calibrate(const Eigen::Isometry3d& init_T_camera_lidar) {
  CalibrationResult result;
  if (dataset.empty()) {
    result.error = CalibrationError::EmptyDataset;
    result.message = "dataset is empty";
    return result;
  }
  for (const auto& data : dataset) {
    if (!data || !data->points || data->points->size() == 0) {
      result.error = CalibrationError::EmptyPointCloud;
      result.message = "dataset contains an empty point cloud";
      return result;
    }
  }

  Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar;
  result.T_camera_lidar = T_camera_lidar;

  for (int outer = 0; outer < params.max_outer_iterations; ++outer) {
    if (params.is_cancelled && params.is_cancelled()) {
      result.error = CalibrationError::Cancelled;
      result.message = "cancelled";
      return result;
    }
    CalibrationResult inner;
    switch (params.registration_type) {
      case RegistrationType::NID_BFGS:
        inner = estimate_pose_bfgs(T_camera_lidar, outer);
        break;
      case RegistrationType::NID_NELDER_MEAD:
        inner = estimate_pose_nelder_mead(T_camera_lidar, outer);
        break;
      default:
        result.error = CalibrationError::InvalidArgument;
        result.message = "unknown registration_type";
        return result;
    }

    if (inner.error != CalibrationError::Ok) {
      return inner;
    }
    if (params.is_cancelled && params.is_cancelled()) {
      result.error = CalibrationError::Cancelled;
      result.message = "cancelled";
      return result;
    }

    const Eigen::Isometry3d delta = inner.T_camera_lidar.inverse() * T_camera_lidar;
    T_camera_lidar = inner.T_camera_lidar;
    result.T_camera_lidar = T_camera_lidar;
    result.final_cost = inner.final_cost;
    result.inner_iterations += inner.inner_iterations;
    result.outer_iterations = outer + 1;

    const double delta_t = delta.translation().norm();
    const double delta_r = Eigen::AngleAxisd(delta.linear()).angle();
    if (delta_t < params.delta_trans_thresh && delta_r < params.delta_rot_thresh) {
      break;
    }
  }

  result.error = CalibrationError::Ok;
  return result;
}

CalibrationResult VisualCameraCalibration::estimate_pose_nelder_mead(const Eigen::Isometry3d& init_T_camera_lidar, int outer_iteration) {
  CalibrationResult result;
  ViewCullingParams view_culling_params;
  view_culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
  ViewCulling view_culling(proj, {dataset.front()->image.cols, dataset.front()->image.rows}, view_culling_params);

  std::vector<CostCalculator::Ptr> costs;
  for (const auto& data : dataset) {
    auto culled_points = view_culling.cull(data->points, init_T_camera_lidar);
    const cv::Mat gray = to_grayscale(data->image);
    auto new_data = std::make_shared<VisualLiDARData>(gray, culled_points);

    NIDCostParams nid_params;
    nid_params.bins = params.nid_bins;
    costs.emplace_back(std::make_shared<CostCalculatorNID>(proj, new_data, nid_params));
  }

  double best_cost = std::numeric_limits<double>::max();

  const auto f = [&](const Eigen::Matrix<double, 6, 1>& x) {
    if (params.is_cancelled && params.is_cancelled()) {
      return best_cost;
    }
    const Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar * expmap_delta(x);
    double sum_costs = 0.0;

#pragma omp parallel for num_threads(params.num_threads) reduction(+ : sum_costs)
    for (int i = 0; i < static_cast<int>(costs.size()); i++) {
      sum_costs += costs[i]->calculate(T_camera_lidar);
    }

    if (sum_costs < best_cost) {
      best_cost = sum_costs;
      if (params.on_progress) {
        CalibrationProgress progress;
        progress.outer_iteration = outer_iteration;
        progress.cost = best_cost;
        progress.T_camera_lidar = T_camera_lidar;
        params.on_progress(progress);
      }
    }

    return sum_costs;
  };

  dfo::NelderMead<6>::Params nelder_mead_params;
  nelder_mead_params.init_step = params.nelder_mead_init_step;
  nelder_mead_params.convergence_var_thresh = params.nelder_mead_convergence_criteria;
  nelder_mead_params.max_iterations = params.max_inner_iterations;
  nelder_mead_params.should_stop = params.is_cancelled;
  dfo::NelderMead<6> optimizer(nelder_mead_params);
  auto opt = optimizer.optimize(f, Eigen::Matrix<double, 6, 1>::Zero());

  if (params.is_cancelled && params.is_cancelled()) {
    result.error = CalibrationError::Cancelled;
    result.message = "cancelled";
    return result;
  }

  result.T_camera_lidar = init_T_camera_lidar * expmap_delta(opt.x);
  result.final_cost = opt.y;
  result.inner_iterations = opt.num_iterations;
  result.error = CalibrationError::Ok;
  return result;
}

struct MultiNIDCost {
public:
  MultiNIDCost(const Sophus::SE3d& init_T_camera_lidar) : init_T_camera_lidar(init_T_camera_lidar) {}

  void add(const std::shared_ptr<NIDCost>& cost) { costs.emplace_back(cost); }

  template <typename T>
  bool operator()(const T* params, T* residual) const {
    std::vector<double> values(Sophus::SE3d::num_parameters);
    std::transform(params, params + Sophus::SE3d::num_parameters, values.begin(), [](const auto& x) { return get_real(x); });
    const Eigen::Map<const Sophus::SE3d> T_camera_lidar(values.data());
    const Sophus::SE3d delta = init_T_camera_lidar.inverse() * T_camera_lidar;

    if (delta.translation().norm() > 0.2 || Eigen::AngleAxisd(delta.rotationMatrix()).angle() > 2.0 * M_PI / 180.0) {
      return false;
    }

    std::vector<bool> results(costs.size());
    std::vector<T> residuals(costs.size());

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(costs.size()); i++) {
      results[i] = (*costs[i])(params, &residuals[i]);
    }

    for (int i = 1; i < static_cast<int>(costs.size()); i++) {
      residuals[0] += residuals[i];
    }

    *residual = residuals[0];
    return std::count(results.begin(), results.end(), false) == 0;
  }

private:
  Sophus::SE3d init_T_camera_lidar;
  std::vector<std::shared_ptr<NIDCost>> costs;
};

struct IterationCallbackWrapper : public ceres::IterationCallback {
public:
  IterationCallbackWrapper(const std::function<ceres::CallbackReturnType(const ceres::IterationSummary&)>& callback) : callback(callback) {}

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override { return callback(summary); }

private:
  std::function<ceres::CallbackReturnType(const ceres::IterationSummary&)> callback;
};

CalibrationResult VisualCameraCalibration::estimate_pose_bfgs(const Eigen::Isometry3d& init_T_camera_lidar, int outer_iteration) {
  CalibrationResult result;
  ViewCullingParams view_culling_params;
  view_culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
  ViewCulling view_culling(proj, {dataset.front()->image.cols, dataset.front()->image.rows}, view_culling_params);

  Sophus::SE3d T_camera_lidar(init_T_camera_lidar.matrix());
  std::vector<std::shared_ptr<NIDCost>> nid_costs;

  for (const auto& data : dataset) {
    auto culled_points = view_culling.cull(data->points, init_T_camera_lidar);
    const cv::Mat gray = to_grayscale(data->image);

    cv::Mat normalized_image;
    gray.convertTo(normalized_image, CV_64FC1, 1.0 / 255.0);

    nid_costs.emplace_back(std::make_shared<NIDCost>(proj, normalized_image, culled_points, params.nid_bins));
  }

  auto sum_nid = new MultiNIDCost(T_camera_lidar);
  for (const auto& nid_cost : nid_costs) {
    sum_nid->add(nid_cost);
  }

  auto cost = new ceres::AutoDiffFirstOrderFunction<MultiNIDCost, Sophus::SE3d::num_parameters>(sum_nid);
  ceres::GradientProblem problem(cost, new Sophus::Manifold<Sophus::SE3>());

  ceres::GradientProblemSolver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.update_state_every_iteration = true;
  options.line_search_direction_type = ceres::BFGS;
  options.max_num_iterations = params.max_inner_iterations;

  if (params.on_progress || params.is_cancelled) {
    options.callbacks.emplace_back(new IterationCallbackWrapper([&](const ceres::IterationSummary& summary) {
      if (params.is_cancelled && params.is_cancelled()) {
        return ceres::CallbackReturnType::SOLVER_ABORT;
      }
      if (params.on_progress) {
        CalibrationProgress progress;
        progress.outer_iteration = outer_iteration;
        progress.inner_iteration = static_cast<int>(summary.iteration);
        progress.cost = summary.cost;
        progress.T_camera_lidar = Eigen::Isometry3d(T_camera_lidar.matrix());
        params.on_progress(progress);
      }
      return ceres::CallbackReturnType::SOLVER_CONTINUE;
    }));
  }

  ceres::GradientProblemSolver::Summary summary;
  ceres::Solve(options, problem, T_camera_lidar.data(), &summary);

  if (params.is_cancelled && params.is_cancelled()) {
    result.error = CalibrationError::Cancelled;
    result.message = "cancelled";
    return result;
  }

  if (!summary.IsSolutionUsable()) {
    result.error = CalibrationError::OptimizationFailed;
    result.message = "BFGS failed to converge";
    return result;
  }

  result.T_camera_lidar = Eigen::Isometry3d(T_camera_lidar.matrix());
  result.final_cost = summary.final_cost;
  result.inner_iterations = static_cast<int>(summary.iterations.size());
  result.error = CalibrationError::Ok;
  return result;
}

}  // namespace vlcal
