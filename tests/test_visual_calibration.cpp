#include <gtest/gtest.h>

#include <atomic>
#include <opencv2/core.hpp>

#include <camera/create_camera.hpp>
#include <vlcal/calib/visual_camera_calibration.hpp>
#include <vlcal/common/frame_cpu.hpp>
#include <vlcal/common/scan_time.hpp>

namespace {

vlcal::VisualLiDARData::Ptr make_synthetic_pair(double yaw_offset_rad) {
  const int w = 64;
  const int h = 48;
  cv::Mat image(h, w, CV_8UC1, cv::Scalar(0));

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
  std::vector<double> intensities;
  for (int y = 8; y < h - 8; ++y) {
    for (int x = 8; x < w - 8; ++x) {
      const double u = (x - w / 2.0) * 0.01;
      const double v = (y - h / 2.0) * 0.01;
      Eigen::Vector4d p(u, v, 2.0, 1.0);
      points.push_back(p);
      intensities.push_back(static_cast<double>(x) / w);
      image.at<uint8_t>(y, x) = static_cast<uint8_t>(255.0 * intensities.back());
    }
  }

  auto frame = std::make_shared<vlcal::FrameCPU>(points);
  frame->add_intensities(intensities);
  return std::make_shared<vlcal::VisualLiDARData>(image, frame);
}

}  // namespace

TEST(VisualCalibration, AcceptsRgbImage) {
  auto data = make_synthetic_pair(0.0);
  cv::Mat bgr(data->image.rows, data->image.cols, CV_8UC3);
  for (int y = 0; y < data->image.rows; ++y) {
    for (int x = 0; x < data->image.cols; ++x) {
      const uint8_t v = data->image.at<uint8_t>(y, x);
      bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(v, v, v);
    }
  }
  data->image = bgr;

  const auto proj = camera::create_camera("plumb_bob", {static_cast<double>(data->image.cols), static_cast<double>(data->image.rows), data->image.cols / 2.0, data->image.rows / 2.0}, {});
  vlcal::VisualCameraCalibrationParams params;
  params.max_outer_iterations = 1;
  params.max_inner_iterations = 5;
  vlcal::VisualCameraCalibration calib(proj, {data}, params);
  const auto result = calib.calibrate(Eigen::Isometry3d::Identity());
  EXPECT_EQ(result.error, vlcal::CalibrationError::Ok);
}

TEST(VisualCalibration, EmptyDatasetFails) {
  const auto proj = camera::create_camera("plumb_bob", {640.0, 480.0, 320.0, 240.0}, {});
  vlcal::VisualCameraCalibration calib(proj, {}, {});
  const auto result = calib.calibrate(Eigen::Isometry3d::Identity());
  EXPECT_EQ(result.error, vlcal::CalibrationError::EmptyDataset);
}

TEST(VisualCalibration, ProgressCallbackNoOp) {
  auto data = make_synthetic_pair(0.05);
  const auto proj = camera::create_camera("plumb_bob", {static_cast<double>(data->image.cols), static_cast<double>(data->image.rows), data->image.cols / 2.0, data->image.rows / 2.0}, {});
  vlcal::VisualCameraCalibrationParams params;
  params.max_outer_iterations = 1;
  params.max_inner_iterations = 8;
  std::atomic<int> events{0};
  params.on_progress = [&](const vlcal::CalibrationProgress&) { events++; };
  vlcal::VisualCameraCalibration calib(proj, {data}, params);
  const auto result = calib.calibrate(Eigen::Isometry3d::Identity());
  EXPECT_EQ(result.error, vlcal::CalibrationError::Ok);
}

TEST(VisualCalibration, CancelledBeforeStart) {
  auto data = make_synthetic_pair(0.0);
  const auto proj = camera::create_camera(
    "plumb_bob",
    {static_cast<double>(data->image.cols), static_cast<double>(data->image.rows), data->image.cols / 2.0, data->image.rows / 2.0},
    {});
  vlcal::VisualCameraCalibrationParams params;
  params.max_outer_iterations = 10;
  params.is_cancelled = []() { return true; };
  vlcal::VisualCameraCalibration calib(proj, {data}, params);
  const auto result = calib.calibrate(Eigen::Isometry3d::Identity());
  EXPECT_EQ(result.error, vlcal::CalibrationError::Cancelled);
}

TEST(ScanTime, AzimuthWindowRequiresDuration) {
  vlcal::ScanTimeParams params;
  params.time_origin = vlcal::TimeOrigin::ScanStart;
  params.scan_duration.reset();
  vlcal::NormalizedScanTimes out;
  const auto err = vlcal::normalize_scan_times({0.25, 0.35, 0.5}, params, out);
  EXPECT_FALSE(err.ok);
}

TEST(ScanTime, AzimuthWindowAlphaRange) {
  vlcal::ScanTimeParams params;
  params.time_origin = vlcal::TimeOrigin::ScanStart;
  params.scan_duration = 1.0;
  vlcal::NormalizedScanTimes out;
  const auto err = vlcal::normalize_scan_times({0.25, 0.35, 0.5}, params, out);
  EXPECT_TRUE(err.ok);
  EXPECT_NEAR(vlcal::scan_time_alpha(out.times.front(), out.scan_duration), 0.25, 1e-6);
  EXPECT_NEAR(vlcal::scan_time_alpha(out.times.back(), out.scan_duration), 0.5, 1e-6);
}
