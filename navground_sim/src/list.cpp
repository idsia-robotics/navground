/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <vector>

#include "navground/core/plugins.h"
#include "navground/core/property.h"
#include "navground/core/utilities.h"
#include "navground/core/behavior.h"
#include "navground/sim/scenario.h"
#include "navground/sim/scenarios/antipodal.h"
#include "navground/sim/scenarios/simple.h"
#include "navground/sim/world.h"

template <typename T>
static void print_register(const std::string& title,
                           const std::string& type = "") {
  std::cout << title << std::endl;
  for (unsigned i = 0; i < title.size(); ++i) {
    std::cout << '=';
  }
  std::cout << std::endl;
  for (const auto& [name, properties] : T::type_properties()) {
    if (!type.empty() && name != type) continue;
    std::cout << "- " << name << std::endl;
    for (const auto& [k, p] : properties) {
      std::cout << "     " << k << ": " << p.default_value << " ["
                << p.type_name << "]" << std::endl;
    }
  }
}

static void show_usage(const std::string& name) {
  std::cout << "Usage: " << name << " <option(s)>" << std::endl
            << "Options:" << std::endl
            << "  --help\t\t\tShow this help message" << std::endl
            << "  --state_estimation\t\t\tShow only state estimations"
            << std::endl
            << "  --state_estimation <BEHAVIOR>\t\tShow only a specific state "
               "estimations"
            << std::endl
            << "  --task\t\t\tShow only tasks" << std::endl
            << "  --task <TASK>\tShow only a specific task" << std::endl
            << "  --world\t\t\tShow only scenarios" << std::endl
            << "  --world <WORLD>\tShow only a specific scenario"
            << std::endl;
}

int main(int argc, char* argv[]) {
  load_plugins();
  std::string state_estimation_type = "";
  std::string task_type = "";
  std::string scenario_type = "";
  bool state_estimations = true;
  bool tasks = true;
  bool scenarios = true;
  for (int i = 0; i < argc; i++) {
    if (state_estimations && std::string(argv[i]) == "--state_estimation") {
      if (i + 1 < argc) {
        state_estimation_type = std::string(argv[i + 1]);
        i++;
      }
      tasks = scenarios = false;
    }
    if (tasks && std::string(argv[i]) == "--task") {
      if (i + 1 < argc) {
        task_type = std::string(argv[i + 1]);
        i++;
      }
      state_estimations = scenarios = false;
    }
    if (scenarios && std::string(argv[i]) == "--scenarios") {
      if (i + 1 < argc) {
        scenario_type = std::string(argv[i + 1]);
        i++;
      }
      state_estimations = tasks = false;
    }
    if (std::string(argv[i]) == "--help") {
      show_usage(argv[0]);
      return 0;
    }
  }
  // print_register<navground::core::Behavior>("Behaviors", "");

  if (state_estimations) {
    print_register<navground::sim::StateEstimation>("State Estimations",
                                                       state_estimation_type);
    std::cout << std::endl;
  }
  if (tasks) {
    print_register<navground::sim::Task>("Tasks", task_type);
    std::cout << std::endl;
  }
  if (scenarios) {
    print_register<navground::sim::Scenario>("Scenarios", scenario_type);
  }
  return 0;
}
