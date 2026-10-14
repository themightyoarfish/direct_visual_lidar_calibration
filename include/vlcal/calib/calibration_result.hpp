#pragma once

#include <Eigen/Geometry>
#include <string>

namespace vlcal {

enum class CalibrationError {
  Ok = 0,
  InvalidArgument,
  EmptyDataset,
  EmptyPointCloud,
  OptimizationFailed,
  Cancelled,
};

struct CalibrationProgress {
  int outer_iteration = 0;
  int inner_iteration = 0;
  double cost = 0.0;
  Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
};

struct CalibrationResult {
  CalibrationError error = CalibrationError::Ok;
  std::string message;
  Eigen::Isometry3d T_camera_lidar = Eigen::Isometry3d::Identity();
  double final_cost = 0.0;
  int outer_iterations = 0;
  int inner_iterations = 0;
};

inline const char* calibration_error_string(CalibrationError e) {
  switch (e) {
    case CalibrationError::Ok:
      return "ok";
    case CalibrationError::InvalidArgument:
      return "invalid_argument";
    case CalibrationError::EmptyDataset:
      return "empty_dataset";
    case CalibrationError::EmptyPointCloud:
      return "empty_point_cloud";
    case CalibrationError::OptimizationFailed:
      return "optimization_failed";
    default:
      return "unknown";
  }
}

}  // namespace vlcal
