#include "tzcup_j6/d1_yolov9_postprocess.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using tzcup_j6::D1PostprocessConfig;
using tzcup_j6::FloatTensorView;
using tzcup_j6::kD1ProposalCount;
using tzcup_j6::kD1TensorElementCount;

std::size_t Offset(std::size_t attribute, std::size_t proposal) {
  return attribute * kD1ProposalCount + proposal;
}

void Require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void SetProposal(std::vector<float>& output, std::size_t proposal,
                 float center_x, float center_y, float width, float height,
                 std::size_t class_index, float score) {
  output[Offset(0, proposal)] = center_x;
  output[Offset(1, proposal)] = center_y;
  output[Offset(2, proposal)] = width;
  output[Offset(3, proposal)] = height;
  output[Offset(4 + class_index, proposal)] = score;
}

FloatTensorView View(const std::vector<float>& output) {
  return FloatTensorView{output.data(), output.size(), {1, 14, 8400}};
}

template <typename Callable>
void ExpectInvalidArgument(Callable callable) {
  bool rejected = false;
  try {
    callable();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Require(rejected, "expected std::invalid_argument");
}

void TestFixedLayoutThresholdAndMapping() {
  std::vector<float> output(kD1TensorElementCount, 0.0F);
  SetProposal(output, 0, 100.0F, 100.0F, 20.0F, 40.0F, 1, 0.90F);
  SetProposal(output, 1, 200.0F, 200.0F, 30.0F, 30.0F, 2, 0.80F);
  SetProposal(output, 2, 300.0F, 300.0F, 10.0F, 10.0F, 7, 0.70F);
  SetProposal(output, 3, 400.0F, 400.0F, 20.0F, 20.0F, 9, 0.60F);
  SetProposal(output, 4, 500.0F, 500.0F, 20.0F, 20.0F, 0, 0.24F);

  const auto detections = tzcup_j6::DecodeD1YoloV9(View(output));
  Require(detections.size() == 4, "threshold did not retain four proposals");
  Require(detections[0].source_class == "plastic_bottle",
          "D1 plastic class name mismatch");
  Require(detections[0].product_class == "plastic_bottle",
          "plastic bottle mapping mismatch");
  Require(detections[1].source_class == "drinks_can",
          "D1 can class name mismatch");
  Require(detections[1].product_class == "metal_can", "can mapping mismatch");
  Require(detections[2].source_class == "paper_waste",
          "D1 paper class name mismatch");
  Require(detections[2].product_class == "paper_litter",
          "paper mapping mismatch");
  Require(detections[3].source_class == "general_litter",
          "D1 class nine name mismatch");
  Require(detections[3].product_class == "background_or_unknown",
          "non-actionable mapping mismatch");
  Require(std::abs(detections[0].bbox_xyxy[0] - 90.0F) < 1.0e-6F,
          "xywh to xyxy conversion mismatch");
}

void TestClassAwareNms() {
  std::vector<float> output(kD1TensorElementCount, 0.0F);
  SetProposal(output, 0, 100.0F, 100.0F, 40.0F, 40.0F, 1, 0.95F);
  SetProposal(output, 1, 101.0F, 101.0F, 40.0F, 40.0F, 1, 0.90F);
  SetProposal(output, 2, 100.0F, 100.0F, 40.0F, 40.0F, 2, 0.85F);

  const auto detections = tzcup_j6::DecodeD1YoloV9(View(output));
  Require(detections.size() == 2, "class-aware NMS result count mismatch");
  Require(detections[0].proposal_index == 0,
          "higher-scoring same-class proposal was not retained");
  Require(detections[1].proposal_index == 2,
          "overlapping different-class proposal was incorrectly suppressed");
}

void TestClassAwareNmsKeepsDifferentUnknownSourceClasses() {
  std::vector<float> output(kD1TensorElementCount, 0.0F);
  SetProposal(output, 0, 100.0F, 100.0F, 40.0F, 40.0F, 0, 0.95F);
  SetProposal(output, 1, 100.0F, 100.0F, 40.0F, 40.0F, 3, 0.90F);

  const auto detections = tzcup_j6::DecodeD1YoloV9(View(output));
  Require(detections.size() == 2,
          "different unknown source classes were incorrectly suppressed");
  Require(detections[0].source_class == "cigarette_butt",
          "first unknown source class mismatch");
  Require(detections[1].source_class == "fast_food_packaging",
          "second unknown source class mismatch");
  Require(detections[0].product_class == "background_or_unknown" &&
              detections[1].product_class == "background_or_unknown",
          "unknown product mapping mismatch");
}

void TestAllNonActionableClassesMapToUnknown() {
  std::vector<float> output(kD1TensorElementCount, 0.0F);
  const std::vector<std::size_t> unknown_classes = {0, 3, 4, 5, 6, 8, 9};
  for (const std::size_t class_index : unknown_classes) {
    const float center = 40.0F + static_cast<float>(class_index) * 50.0F;
    SetProposal(output, class_index, center, center, 20.0F, 20.0F,
                class_index, 0.90F);
  }
  const auto detections = tzcup_j6::DecodeD1YoloV9(View(output));
  Require(detections.size() == 7,
          "all seven non-actionable D1 classes must remain observable");
  for (const auto& detection : detections) {
    Require(detection.source_class_index != 1 &&
                detection.source_class_index != 2 &&
                detection.source_class_index != 7,
            "unexpected actionable class in unknown mapping test");
    Require(detection.product_class == "background_or_unknown",
            "non-actionable D1 class did not map to unknown");
  }
}

void TestShapeAndNonFiniteFailClosed() {
  std::vector<float> output(kD1TensorElementCount, 0.0F);
  ExpectInvalidArgument([&output]() {
    tzcup_j6::DecodeD1YoloV9(
        FloatTensorView{output.data(), output.size(), {1, 8400, 14}});
  });
  ExpectInvalidArgument([&output]() {
    tzcup_j6::DecodeD1YoloV9(
        FloatTensorView{output.data(), output.size() - 1, {1, 14, 8400}});
  });
  ExpectInvalidArgument([]() {
    tzcup_j6::DecodeD1YoloV9(
        FloatTensorView{nullptr, kD1TensorElementCount, {1, 14, 8400}});
  });
  output[Offset(7, 100)] = std::numeric_limits<float>::quiet_NaN();
  ExpectInvalidArgument([&output]() { tzcup_j6::DecodeD1YoloV9(View(output)); });
  output[Offset(7, 100)] = 0.0F;
  output[Offset(0, 100)] = std::numeric_limits<float>::infinity();
  ExpectInvalidArgument([&output]() { tzcup_j6::DecodeD1YoloV9(View(output)); });
}

void TestInvalidConfigurationAndScoreFailClosed() {
  std::vector<float> output(kD1TensorElementCount, 0.0F);
  D1PostprocessConfig config;
  config.score_threshold = std::numeric_limits<float>::quiet_NaN();
  ExpectInvalidArgument([&output, &config]() {
    tzcup_j6::DecodeD1YoloV9(View(output), config);
  });
  config = D1PostprocessConfig{};
  output[Offset(4, 0)] = 1.01F;
  ExpectInvalidArgument([&output, &config]() {
    tzcup_j6::DecodeD1YoloV9(View(output), config);
  });
}

}  // namespace

int main() {
  TestFixedLayoutThresholdAndMapping();
  TestClassAwareNms();
  TestClassAwareNmsKeepsDifferentUnknownSourceClasses();
  TestAllNonActionableClassesMapToUnknown();
  TestShapeAndNonFiniteFailClosed();
  TestInvalidConfigurationAndScoreFailClosed();
  return 0;
}
