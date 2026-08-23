#include "tzcup_j6/d1_yolov9_postprocess.hpp"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void Require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void RequireNear(float actual, float expected, const std::string& field) {
  Require(std::abs(actual - expected) <= 1.0e-4F,
          field + " differs from the Python golden");
}

std::vector<float> ReadTensor(const std::string& path) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  Require(stream.good(), "cannot open external D1 golden tensor");
  const std::streamsize byte_count = stream.tellg();
  Require(byte_count ==
              static_cast<std::streamsize>(tzcup_j6::kD1TensorElementCount *
                                           sizeof(float)),
          "external D1 golden tensor byte count mismatch");
  stream.seekg(0, std::ios::beg);
  std::vector<float> values(tzcup_j6::kD1TensorElementCount);
  stream.read(reinterpret_cast<char*>(values.data()), byte_count);
  Require(stream.good(), "failed to read external D1 golden tensor");
  return values;
}

}  // namespace

int main(int argc, char** argv) {
  Require(argc == 2, "expected external D1 golden tensor path");
  const std::vector<float> values = ReadTensor(argv[1]);
  const auto detections = tzcup_j6::DecodeD1YoloV9(
      tzcup_j6::FloatTensorView{values.data(), values.size(), {1, 14, 8400}});
  Require(detections.size() == 2, "real D1 golden detection count mismatch");
  for (const auto& detection : detections) {
    Require(detection.source_class_index == 9,
            "real D1 golden source class index mismatch");
    Require(detection.source_class == "general_litter",
            "real D1 golden source class mismatch");
    Require(detection.product_class == "background_or_unknown",
            "real D1 golden product mapping mismatch");
  }
  RequireNear(detections[0].score, 0.8742088675498962F, "detection[0].score");
  RequireNear(detections[0].bbox_xyxy[0], 0.0F, "detection[0].x1");
  RequireNear(detections[0].bbox_xyxy[1], 78.56005859375F, "detection[0].y1");
  RequireNear(detections[0].bbox_xyxy[2], 639.722412109375F, "detection[0].x2");
  RequireNear(detections[0].bbox_xyxy[3], 441.71563720703125F, "detection[0].y2");
  RequireNear(detections[1].score, 0.7494193315505981F, "detection[1].score");
  RequireNear(detections[1].bbox_xyxy[0], 0.0F, "detection[1].x1");
  RequireNear(detections[1].bbox_xyxy[1], 428.32679748535156F, "detection[1].y1");
  RequireNear(detections[1].bbox_xyxy[2], 640.0F, "detection[1].x2");
  RequireNear(detections[1].bbox_xyxy[3], 568.1257171630859F, "detection[1].y2");
  return 0;
}
