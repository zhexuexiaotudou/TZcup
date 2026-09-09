#include "tzcup_j6/d1_yolov9_postprocess.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace tzcup_j6 {
namespace {

static_assert(sizeof(float) == 4, "D1 postprocess requires 32-bit float");
static_assert(std::numeric_limits<float>::is_iec559,
              "D1 postprocess requires IEEE-754 FP32");

constexpr std::array<std::string_view, kD1ClassCount> kSourceClasses = {
    "cigarette_butt",       "plastic_bottle", "drinks_can",
    "fast_food_packaging",  "plastic_bag",    "coffee_cup",
    "glass_bottle",         "paper_waste",    "food_wrapper",
    "general_litter",
};

constexpr std::array<std::string_view, kD1ClassCount> kProductClasses = {
    "background_or_unknown", "plastic_bottle",        "metal_can",
    "background_or_unknown", "background_or_unknown", "background_or_unknown",
    "background_or_unknown", "paper_litter",           "background_or_unknown",
    "background_or_unknown",
};

float ValueAt(const FloatTensorView& tensor, std::size_t attribute,
              std::size_t proposal) {
  return tensor.data[attribute * kD1ProposalCount + proposal];
}

double IntersectionOverUnion(const D1Detection& first,
                             const D1Detection& second) {
  const double x1 = std::max(first.bbox_xyxy[0], second.bbox_xyxy[0]);
  const double y1 = std::max(first.bbox_xyxy[1], second.bbox_xyxy[1]);
  const double x2 = std::min(first.bbox_xyxy[2], second.bbox_xyxy[2]);
  const double y2 = std::min(first.bbox_xyxy[3], second.bbox_xyxy[3]);
  const double intersection =
      std::max(0.0, x2 - x1) * std::max(0.0, y2 - y1);
  const double first_area =
      std::max(0.0, static_cast<double>(first.bbox_xyxy[2]) -
                        first.bbox_xyxy[0]) *
      std::max(0.0, static_cast<double>(first.bbox_xyxy[3]) -
                        first.bbox_xyxy[1]);
  const double second_area =
      std::max(0.0, static_cast<double>(second.bbox_xyxy[2]) -
                        second.bbox_xyxy[0]) *
      std::max(0.0, static_cast<double>(second.bbox_xyxy[3]) -
                        second.bbox_xyxy[1]);
  const double union_area = first_area + second_area - intersection;
  return union_area > 0.0 ? intersection / union_area : 0.0;
}

void ValidateTensor(const FloatTensorView& tensor) {
  const std::vector<std::size_t> expected_shape = {
      kD1Batch, kD1Attributes, kD1ProposalCount};
  if (tensor.data == nullptr) {
    throw std::invalid_argument("D1 output data pointer is null");
  }
  if (tensor.shape != expected_shape ||
      tensor.element_count != kD1TensorElementCount) {
    throw std::invalid_argument(
        "D1 output must be FP32 with shape [1,14,8400]");
  }
  for (std::size_t index = 0; index < tensor.element_count; ++index) {
    if (!std::isfinite(tensor.data[index])) {
      throw std::invalid_argument("D1 output contains a non-finite value");
    }
  }
}

}  // namespace

void D1PostprocessConfig::Validate() const {
  if (!std::isfinite(score_threshold) || score_threshold < 0.0F ||
      score_threshold > 1.0F || !std::isfinite(nms_iou_threshold) ||
      nms_iou_threshold < 0.0F || nms_iou_threshold > 1.0F ||
      !std::isfinite(input_width) || input_width <= 0.0F ||
      !std::isfinite(input_height) || input_height <= 0.0F ||
      maximum_detections == 0) {
    throw std::invalid_argument("D1 postprocess configuration is invalid");
  }
}

std::vector<D1Detection> DecodeD1YoloV9(
    const FloatTensorView& tensor, const D1PostprocessConfig& config) {
  config.Validate();
  ValidateTensor(tensor);

  std::vector<D1Detection> candidates;
  candidates.reserve(kD1ProposalCount);
  for (std::size_t proposal = 0; proposal < kD1ProposalCount; ++proposal) {
    std::size_t class_index = 0;
    float class_score = ValueAt(tensor, 4, proposal);
    for (std::size_t index = 1; index < kD1ClassCount; ++index) {
      const float score = ValueAt(tensor, 4 + index, proposal);
      if (score > class_score) {
        class_score = score;
        class_index = index;
      }
    }
    if (class_score < 0.0F || class_score > 1.0F) {
      throw std::invalid_argument("D1 class score is outside [0,1]");
    }
    if (class_score < config.score_threshold) {
      continue;
    }

    const double center_x = ValueAt(tensor, 0, proposal);
    const double center_y = ValueAt(tensor, 1, proposal);
    const double width = ValueAt(tensor, 2, proposal);
    const double height = ValueAt(tensor, 3, proposal);
    if (width <= 0.0 || height <= 0.0) {
      continue;
    }
    const double x1 = std::clamp(center_x - width * 0.5, 0.0,
                                 static_cast<double>(config.input_width));
    const double y1 = std::clamp(center_y - height * 0.5, 0.0,
                                 static_cast<double>(config.input_height));
    const double x2 = std::clamp(center_x + width * 0.5, 0.0,
                                 static_cast<double>(config.input_width));
    const double y2 = std::clamp(center_y + height * 0.5, 0.0,
                                 static_cast<double>(config.input_height));
    if (x2 <= x1 || y2 <= y1) {
      continue;
    }
    candidates.push_back(D1Detection{
        {static_cast<float>(x1), static_cast<float>(y1),
         static_cast<float>(x2), static_cast<float>(y2)},
        class_score,
        class_index,
        proposal,
        std::string(kSourceClasses[class_index]),
        std::string(kProductClasses[class_index]),
    });
  }

  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const D1Detection& first, const D1Detection& second) {
                     if (first.score != second.score) {
                       return first.score > second.score;
                     }
                     return first.proposal_index < second.proposal_index;
                   });
  std::vector<D1Detection> kept;
  kept.reserve(std::min(config.maximum_detections, candidates.size()));
  for (const D1Detection& candidate : candidates) {
    const bool suppressed = std::any_of(
        kept.begin(), kept.end(), [&candidate, &config](const D1Detection& prior) {
          return prior.source_class_index == candidate.source_class_index &&
                 IntersectionOverUnion(candidate, prior) >=
                     config.nms_iou_threshold;
        });
    if (!suppressed) {
      kept.push_back(candidate);
      if (kept.size() == config.maximum_detections) {
        break;
      }
    }
  }
  return kept;
}

}  // namespace tzcup_j6
