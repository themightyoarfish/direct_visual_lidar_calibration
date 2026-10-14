#pragma once

#include <functional>
#include <vector>

#include <opencv2/core.hpp>

#include <camera/generic_camera_base.hpp>
#include <vlcal/calib/calibration_result.hpp>
#include <vlcal/common/visual_lidar_data.hpp>

namespace vlcal {

enum class RegistrationType { NID_BFGS, NID_NELDER_MEAD };

struct VisualCameraCalibrationParams {
public:
  VisualCameraCalibrationParams() {
    max_outer_iterations = 10;
    max_inner_iterations = 256;
    num_threads = 4;

    delta_trans_thresh = 0.1;
    delta_rot_thresh = 0.5 * M_PI / 180.0;

    disable_z_buffer_culling = false;

    nid_bins = 16;

    registration_type = RegistrationType::NID_BFGS;
    nelder_mead_init_step = 1e-3;
    nelder_mead_convergence_criteria = 1e-8;
  }

  int max_outer_iterations;
  int max_inner_iterations;
  int num_threads;
  double delta_trans_thresh;
  double delta_rot_thresh;

  bool disable_z_buffer_culling;
  int nid_bins;

  RegistrationType registration_type;
  double nelder_mead_init_step;
  double nelder_mead_convergence_criteria;

  std::function<void(const CalibrationProgress&)> on_progress;
  /// When set, return true if calibration should stop as soon as practical.
  std::function<bool()> is_cancelled;
};

class VisualCameraCalibration {
public:
  VisualCameraCalibration(
    const camera::GenericCameraBase::ConstPtr& proj,
    const std::vector<VisualLiDARData::ConstPtr>& dataset,
    const VisualCameraCalibrationParams& params = VisualCameraCalibrationParams());

  CalibrationResult calibrate(const Eigen::Isometry3d& init_T_camera_lidar);

private:
  CalibrationResult estimate_pose_nelder_mead(const Eigen::Isometry3d& init_T_camera_lidar, int outer_iteration);
  CalibrationResult estimate_pose_bfgs(const Eigen::Isometry3d& init_T_camera_lidar, int outer_iteration);

  static cv::Mat to_grayscale(const cv::Mat& image);

private:
  const VisualCameraCalibrationParams params;
  const camera::GenericCameraBase::ConstPtr proj;
  const std::vector<VisualLiDARData::ConstPtr> dataset;
};

}  // namespace vlcal
