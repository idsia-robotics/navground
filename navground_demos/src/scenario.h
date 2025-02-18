/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_DEMO_SCENARIO_H
#define NAVGROUND_CORE_DEMO_SCENARIO_H

#include <memory>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/kinematics.h"
#include "navground/sim/experiment.h"
#include "navground/sim/scenario.h"
#include "navground/sim/state_estimations/geometric_bounded.h"
#include "navground/sim/tasks/waypoints.h"
#include "navground/sim/world.h"

namespace core = navground::core;
namespace sim = navground::sim;

struct ThymioDemo : public sim::Scenario {
  static const std::string type;
  explicit ThymioDemo(const std::string &behavior_type = "HL")
      : sim::Scenario(), behavior_type(behavior_type) {}

  void
  init_world(sim::World *world,
             [[maybe_unused]] std::optional<int> seed = std::nullopt) override {
    const std::vector<Vector2> targets{{1, 0}, {-1, 0}};
    for (size_t i = 0; i < 2; i++) {
      auto task =
          std::make_shared<sim::WaypointsTask>(targets, true, (ng_float_t)0.2);
      auto se = std::make_shared<sim::BoundedStateEstimation>(1);
      auto kinematics =
          std::make_shared<core::TwoWheelsDifferentialDriveKinematics>(
              (ng_float_t)0.166, (ng_float_t)0.094);
      auto behavior = core::Behavior::make_type(behavior_type);
      auto agent = sim::Agent::make((ng_float_t)0.08, behavior, kinematics,
                                    task, {se}, (ng_float_t)0.02);
      behavior->set_optimal_speed((ng_float_t)0.12);
      behavior->set_horizon(1);
      behavior->set_safety_margin((ng_float_t)0.02);
      agent->get_controller()->set_speed_tolerance((ng_float_t)0.01);
      agent->pose = {{i ? -0.5 : 0.5, 0.5}, 0};
      agent->type = "thymio";
      world->add_agent(agent);
    }
    world->add_obstacle(sim::Obstacle{{0, 0}, 0.5});
  }
  void set_behavior_type(const std::string &value) { behavior_type = value; }
  std::string get_behavior_type() const { return behavior_type; }

private:
  std::string behavior_type;
};

#endif // NAVGROUND_CORE_DEMO_SCENARIO_H
