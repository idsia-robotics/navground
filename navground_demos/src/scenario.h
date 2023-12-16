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

using navground::core::Behavior;
using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;
using navground::core::TwoWheelsDifferentialDriveKinematics;
using navground::core::Vector2;
namespace sim = navground::sim;

struct ThymioDemo : public sim::Scenario {
  explicit ThymioDemo(const std::string &behavior_type = "HL")
      : sim::Scenario(), behavior_type(behavior_type) {}

  void init_world(sim::World *world, [[maybe_unused]] std::optional<int> seed = std::nullopt) override {
    const std::vector<Vector2> targets{{1, 0}, {-1.0, 0}};
    for (size_t i = 0; i < 2; i++) {
      auto task = std::make_shared<sim::WaypointsTask>(targets, true, 0.2);
      auto se = std::make_shared<sim::BoundedStateEstimation>(1.0);
      auto kinematics = std::make_shared<TwoWheelsDifferentialDriveKinematics>(0.166, 0.094);
      auto behavior = Behavior::make_type(behavior_type);
      auto agent = sim::Agent::make(0.08, behavior, kinematics, task, se, 0.02);
      behavior->set_optimal_speed(0.12);
      behavior->set_horizon(1.0);
      behavior->set_safety_margin(0.02);
      agent->get_controller()->set_speed_tolerance(0.01);
      agent->pose = {{i ? -0.5 : 0.5, 0.5}, 0};
      agent->type = "thymio";
      world->add_agent(agent);
    }
    world->add_obstacle(sim::Obstacle{{0, 0}, 0.5});
  }

  void set_behavior_type(const std::string &value) { behavior_type = value; }
  std::string get_behavior_type() const { return behavior_type; }

  const Properties &get_properties() const override { return properties; };

  inline const static std::map<std::string, Property> properties =
      Properties{{"behavior", make_property<std::string, ThymioDemo>(
                                  &ThymioDemo::get_behavior_type,
                                  &ThymioDemo::set_behavior_type, "HL",
                                  "The navigation behavior")}};

  std::string get_type() const override { return type; }
  inline const static std::string type =
      register_type<ThymioDemo>("ThymioDemo");

 private:
  std::string behavior_type;
};

#endif  // NAVGROUND_CORE_DEMO_SCENARIO_H
