/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/world.h"

#include <memory>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/kinematics.h"
#include "navground/sim/sampler.h"

using navground::core::TwoWheelsDifferentialDriveKinematics;
using navground::core::Vector2;
using navground::sim::BoundedStateEstimation;
using navground::sim::WaypointsTask;
using navground::sim::World;

 using namespace std::placeholders;

class ThymioDemo : public WorldSampler {
 public:

  void init(World * world) {
    const std::vector<Vector2> targets{{1.0f, 0.0f}, {-1.0f, 0.0f}};
    for (size_t i = 0; i < 2; i++) {
      auto task = std::make_shared<WaypointsTask>(targets, true, 0.2);
      auto se = std::make_shared<BoundedStateEstimation>(1.0);
      auto kinematics = std::make_shared<TwoWheelsDifferentialDriveKinematics>(0.166, 0.094);
      auto behavior = Behavior::make_type(behavior_type);
      auto agent = Agent::make(0.08, behavior, kinematics, task, se, 0.02);
      agent->behavior->set_optimal_speed(0.12);
      agent->behavior->set_horizon(1.0);
      agent->behavior->set_safety_margin(0.02);
      agent->controller.set_speed_tolerance(0.01);
      agent->pose = {{i ? -0.5f : 0.5f, 0.5f}, 0.0f};
      world->add_agent(agent);
    }
    world->obstacles.emplace_back(Vector2{0.0f, 0.0f}, 0.1f);
  }

  explicit ThymioDemo(const std::string &behavior_type = "HL")
      : 
      // WorldSampler({&ThymioDemo::init}), 
      WorldSampler({std::bind(&ThymioDemo::init, this, _1)}), 
      behavior_type(behavior_type) {}

  const Properties &get_properties() const override { return properties; };

  std::string get_behavior_type() const { return behavior_type; }

  void set_behavior_type(const std::string &value) { behavior_type = value; }

  std::string get_type() const override { return type; }

  inline const static std::map<std::string, Property> properties =
      Properties{{"behavior_name",
                  make_property<std::string, ThymioDemo>(
                      &ThymioDemo::get_behavior_type,
                      &ThymioDemo::set_behavior_type, "HL", "Behavior name")}};

  inline const static std::string type =
      register_type<ThymioDemo>("ThymioDemo");

 private:
  std::string behavior_type;
};
