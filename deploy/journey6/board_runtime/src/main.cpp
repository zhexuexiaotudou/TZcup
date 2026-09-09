#include "tzcup_j6/contracts.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--self-test") {
    try {
      const tzcup_j6::RuntimeProfile profile{"journey6", "fixture", "nash-e", "fixture"};
      profile.Validate();
      std::cout << "{\"contract_self_test_pass\":true,\"official_runtime_loaded\":false}\n";
      return 0;
    } catch (const std::exception& error) {
      std::cerr << error.what() << '\n';
      return 2;
    }
  }
  std::cerr << "Official Journey 6 HUCP/DNN adapter is not linked. "
               "Build this target inside the board-matched SDK and provide an exact runtime profile.\n";
  return 2;
}
