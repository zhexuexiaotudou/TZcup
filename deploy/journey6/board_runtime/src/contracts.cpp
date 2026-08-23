#include "tzcup_j6/contracts.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace tzcup_j6 {
namespace {

bool SupportedMarch(const std::string& march) {
  return march == "nash-e" || march == "nash-m" || march == "nash-p";
}

}  // namespace

void RuntimeProfile::Validate() const {
  if (target_family != "journey6") {
    throw std::invalid_argument("runtime target family is not Journey 6");
  }
  if (target_sku.empty() || target_sku == "auto") {
    throw std::invalid_argument("runtime requires an inventory-resolved SKU");
  }
  if (!SupportedMarch(target_march)) {
    throw std::invalid_argument("runtime requires a supported inventory-resolved march");
  }
  if (runtime_version.empty() || runtime_version == "auto") {
    throw std::invalid_argument("runtime requires an exact official runtime version");
  }
}

void AckermannCommand::Validate() const {
  if (!std::isfinite(stamp_s) || !std::isfinite(speed_mps) ||
      !std::isfinite(steering_angle_rad) ||
      !std::isfinite(acceleration_limit_mps2) ||
      !std::isfinite(valid_until_s)) {
    throw std::invalid_argument("command contains a non-finite value");
  }
  if (source_id.empty() || acceleration_limit_mps2 <= 0.0 || valid_until_s <= stamp_s) {
    throw std::invalid_argument("command contract is invalid");
  }
}

CommandAuthority::CommandAuthority(std::string source_id, double maximum_future_skew_s)
    : source_id_(std::move(source_id)),
      maximum_future_skew_s_(maximum_future_skew_s),
      last_stamp_s_(-std::numeric_limits<double>::infinity()) {
  if (source_id_.empty() || maximum_future_skew_s_ < 0.0) {
    throw std::invalid_argument("command authority configuration is invalid");
  }
}

void CommandAuthority::Accept(const AckermannCommand& command, double now_s) {
  command.Validate();
  if (!connected_ || !resume_authorized_) {
    throw std::runtime_error("operator resume is required after network loss");
  }
  if (command.source_id != source_id_) {
    throw std::invalid_argument("command source is not the Journey 6 authority");
  }
  if ((has_sequence_ && command.sequence <= last_sequence_) || command.stamp_s <= last_stamp_s_) {
    throw std::invalid_argument("stale or replayed command");
  }
  if (command.stamp_s > now_s + maximum_future_skew_s_ || command.valid_until_s <= now_s) {
    throw std::invalid_argument("command timestamp window is invalid");
  }
  last_sequence_ = command.sequence;
  has_sequence_ = true;
  last_stamp_s_ = command.stamp_s;
  last_command_ = command;
}

void CommandAuthority::NetworkLost() {
  connected_ = false;
  resume_authorized_ = false;
  last_command_.reset();
}

void CommandAuthority::NetworkRestored() { connected_ = true; }

void CommandAuthority::OperatorResume() {
  if (!connected_) {
    throw std::runtime_error("cannot resume a disconnected HIL link");
  }
  resume_authorized_ = true;
  last_command_.reset();
}

std::optional<AckermannCommand> CommandAuthority::Output(double now_s) const {
  if (!connected_ || !resume_authorized_ || !last_command_.has_value() ||
      now_s >= last_command_->valid_until_s) {
    return std::nullopt;
  }
  return last_command_;
}

}  // namespace tzcup_j6
