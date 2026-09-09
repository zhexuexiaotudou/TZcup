#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace tzcup_j6 {

inline constexpr std::size_t kD1Batch = 1;
inline constexpr std::size_t kD1Attributes = 14;
inline constexpr std::size_t kD1ClassCount = 10;
inline constexpr std::size_t kD1ProposalCount = 8400;
inline constexpr std::size_t kD1TensorElementCount =
    kD1Batch * kD1Attributes * kD1ProposalCount;

struct FloatTensorView {
  const float* data{};
  std::size_t element_count{};
  std::vector<std::size_t> shape;
};

struct D1PostprocessConfig {
  float score_threshold{0.25F};
  float nms_iou_threshold{0.45F};
  float input_width{640.0F};
  float input_height{640.0F};
  std::size_t maximum_detections{100};

  void Validate() const;
};

struct D1Detection {
  std::array<float, 4> bbox_xyxy{};
  float score{};
  std::size_t source_class_index{};
  std::size_t proposal_index{};
  std::string source_class;
  std::string product_class;
};

// Decodes the graph-external D1 YOLOv9 output. The only accepted tensor
// contract is FP32 [1, 14, 8400]: xywh plus ten class scores, no objectness.
std::vector<D1Detection> DecodeD1YoloV9(
    const FloatTensorView& tensor,
    const D1PostprocessConfig& config = D1PostprocessConfig{});

}  // namespace tzcup_j6
