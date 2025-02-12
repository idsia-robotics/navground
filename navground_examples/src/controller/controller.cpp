/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <iterator>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/controller.h"

using navground::core::Action;
using navground::core::Behavior;
using navground::core::Controller;
using navground::core::Twist2;
using navground::core::TwoWheelsDifferentialDriveKinematics;

static void show_usage(const std::string &name) {
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
  Controller controller;
  const auto dt = static_cast<ng_float_t>(0.1);
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
  behavior->set_kinematics(
      std::make_shared<TwoWheelsDifferentialDriveKinematics>(
          static_cast<ng_float_t>(1), static_cast<ng_float_t>(0)));
  behavior->set_radius(static_cast<ng_float_t>(0.1));
  controller.set_behavior(behavior);
  controller.set_speed_tolerance(static_cast<ng_float_t>(0.05));
  controller.set_cmd_cb(
      [&, behavior, dt](const Twist2 &cmd) { behavior->actuate(cmd, dt); });
  behavior->set_horizon(1.0);
  const auto &r = *behavior.get();
  printf("Use behavior %s - %s\n", behavior_name, typeid(r).name());
  behavior->set_position({0, 0});
  behavior->prepare();
  // behavior->set_orientation(0.5f);
  // Go to 1, 0, -1.5
  const auto action =
      controller.go_to_position({-1, 0}, static_cast<ng_float_t>(0.1));
  action->done_cb = [](Action::State state) {
    if (state == Action::State::success) {
      printf("Arrived\n");
    }
  };
  action->running_cb = [](ng_float_t time) {
    printf("In progress ... expected to last at least %.2f s\n", time);
  };
  ng_float_t t = 0;
  auto p = behavior->get_position();
  printf("Start loop @ (%.3f, %.3f)\n", p.x(), p.y());
  while (!action->done()) {
    controller.update(dt);
    t += dt;
  }
  p = behavior->get_position();
  const auto v = behavior->get_velocity();
  printf("\nEnd loop after %.1f s @ (%.3f, %.3f), (%.3f, %.3f)\n", t, p.x(),
         p.y(), v.x(), v.y());
  behavior->close();
  return 0;
}
