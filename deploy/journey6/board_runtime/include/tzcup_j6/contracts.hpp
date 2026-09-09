#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace tzcup_j6 {

struct RuntimeProfile {
  std::string target_family;
  std::string target_sku;
  std::string target_march;
  std::string runtime_version;

  void Validate() const;
};

struct AckermannCommand {
  double stamp_s{};
  std::uint64_t sequence{};
  double speed_mps{};
  double steering_angle_rad{};
  double acceleration_limit_mps2{};
  std::string source_id;
  double valid_until_s{};

  void Validate() const;
};

class CommandAuthority {
 public:
  explicit CommandAuthority(std::string source_id, double maximum_future_skew_s = 0.05);

  void Accept(const AckermannCommand& command, double now_s);
  void NetworkLost();
  void NetworkRestored();
  void OperatorResume();
  std::optional<AckermannCommand> Output(double now_s) const;

 private:
  std::string source_id_;
  double maximum_future_skew_s_;
  std::uint64_t last_sequence_{};
  double last_stamp_s_;
  bool has_sequence_{false};
  bool connected_{true};
  bool resume_authorized_{true};
  std::optional<AckermannCommand> last_command_;
};

}  // namespace tzcup_j6
