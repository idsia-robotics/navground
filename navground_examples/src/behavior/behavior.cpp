/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <iterator>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/states/geometric.h"
#include "navground/core/target.h"

// Should also add the wheels

using navground::core::Behavior;
using navground::core::Disc;
using navground::core::GeometricState;
using navground::core::OmnidirectionalKinematics;
using navground::core::Target;

static void show_usage(std::string name) {
  std::vector<std::string> keys = Behavior::types();
  std::ostringstream behaviors;
  // Dump all keys
  std::copy(keys.begin(), keys.end(),
            std::ostream_iterator<std::string>(behaviors, ", "));
  std::cout
      << "Usage: " << name << " <option(s)>" << std::endl
      << "Options:" << std::endl
      << "  --help\t\t\tShow this help message" << std::endl
      << "  --behavior=<NAME>\t\tObstacle avoidance algorithm name: one of "
      << behaviors.str() << std::endl;
}

int main(int argc, char *argv[]) {
  char behavior_name[10] = "HL";
  for (int i = 0; i < argc; i++) {
    if (sscanf(argv[i], "--behavior=%9s", behavior_name)) {
      continue;
    }
    if (strcmp(argv[i], "--help") == 0) {
      show_usage(argv[0]);
      return 0;
    }
  }
  auto behavior = Behavior::make_type(behavior_name);
  if (!behavior) {
    printf("No behavior with name %s\n", behavior_name);
    exit(1);
  }
  printf("Use behavior %s\n", behavior_name);
  behavior->set_kinematics(std::make_shared<OmnidirectionalKinematics>(
      static_cast<ng_float_t>(1), static_cast<ng_float_t>(1)));
  behavior->set_radius(static_cast<ng_float_t>(0.1));
  const auto dt = static_cast<ng_float_t>(0.1);
  behavior->set_horizon(5);
  //  behavior->set_heading_behavior(Behavior::Heading::idle);
  behavior->set_position({0, 0.05});
  // Go to 10, 0
  behavior->set_target(Target::Point({10, 0}));

  GeometricState *geometric_state =
      dynamic_cast<GeometricState *>(behavior->get_environment_state());
  if (geometric_state) {
    geometric_state->set_static_obstacles({Disc({1.5, 0}, 0.5)});
  }
  behavior->prepare();

  // std::cout << *geometric_state << std::endl;

  auto pose = behavior->get_pose();
  auto twist = behavior->get_twist();
  std::cout << "Start loop @ " << pose << " - " << twist << std::endl;
  for (size_t i = 0; i < 30; i++) {
    // pose = behavior->get_pose();
    // printf("%.2f, %.2f, %.2f, ", pose.position.x(), pose.position.y(),
    // pose.orientation);
    const auto cmd = behavior->compute_cmd(dt);
    behavior->actuate(cmd, dt);
  }
  pose = behavior->get_pose();
  twist = behavior->get_twist();
  behavior->close();
  std::cout << "End loop @ " << pose << " - " << twist << std::endl;
  return 0;
}
