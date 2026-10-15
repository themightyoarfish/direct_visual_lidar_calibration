#pragma once

#include <opencv2/core.hpp>

namespace vlcal {

/// True when `mask` is empty (no masking) or pixel (x, y) is non-zero (255 = include in NID).
inline bool image_mask_includes_pixel(const cv::Mat& mask, int x, int y) {
  if (mask.empty()) {
    return true;
  }
  if (x < 0 || y < 0 || x >= mask.cols || y >= mask.rows) {
    return false;
  }
  return mask.at<std::uint8_t>(y, x) != 0;
}

}  // namespace vlcal
