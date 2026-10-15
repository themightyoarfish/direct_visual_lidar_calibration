#pragma once

#include <memory>
#include <iostream>
#include <opencv2/core.hpp>
#include <vlcal/common/frame_cpu.hpp>

namespace vlcal {

struct VisualLiDARData {
public:
  using Ptr = std::shared_ptr<VisualLiDARData>;
  using ConstPtr = std::shared_ptr<const VisualLiDARData>;

  VisualLiDARData() = default;
  VisualLiDARData(const cv::Mat& image, const FrameCPU::Ptr& points) : image(image), points(points) {}
  ~VisualLiDARData() = default;

public:
  cv::Mat image;
  FrameCPU::Ptr points;
  /// Optional CV_8UC1 mask, same size as `image`; 255 = pixel contributes to NID, 0 = excluded.
  cv::Mat image_mask;
};

}  // namespace vlcal
