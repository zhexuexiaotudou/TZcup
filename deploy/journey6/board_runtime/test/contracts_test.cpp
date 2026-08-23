#include "tzcup_j6/contracts.hpp"

#include <cassert>
#include <stdexcept>

int main() {
  using tzcup_j6::AckermannCommand;
  using tzcup_j6::CommandAuthority;
  using tzcup_j6::RuntimeProfile;

  RuntimeProfile{"journey6", "journey6e", "nash-e", "1.2.3"}.Validate();
  bool rejected_auto = false;
  try {
    RuntimeProfile{"journey6", "auto", "auto", "auto"}.Validate();
  } catch (const std::invalid_argument&) {
    rejected_auto = true;
  }
  assert(rejected_auto);

  CommandAuthority authority("j6-algorithm");
  const AckermannCommand command{1.0, 1, 0.5, 0.1, 0.5, "j6-algorithm", 1.2};
  authority.Accept(command, 1.0);
  assert(authority.Output(1.1).has_value());
  assert(!authority.Output(1.2).has_value());
  authority.NetworkLost();
  authority.NetworkRestored();
  bool rejected_without_resume = false;
  try {
    authority.Accept(AckermannCommand{1.1, 2, 0.5, 0.1, 0.5, "j6-algorithm", 1.3}, 1.1);
  } catch (const std::runtime_error&) {
    rejected_without_resume = true;
  }
  assert(rejected_without_resume);
  authority.OperatorResume();
  authority.Accept(AckermannCommand{1.1, 2, 0.5, 0.1, 0.5, "j6-algorithm", 1.3}, 1.1);
  assert(authority.Output(1.2).has_value());
  return 0;
}
