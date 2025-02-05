/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/world.h"

#include <iostream>

#include "navground/core/behaviors/dummy.h"
#include "navground/core/kinematics.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/yaml/world.h"
#include "navground/sim/tasks/waypoints.h"

using navground::core::DummyBehavior;
using navground::core::OmnidirectionalKinematics;
using navground::core::Vector2;
namespace sim = navground::sim;

int main() {
  sim::World world;
  world.add_wall(sim::Wall{Vector2{-1.0, -1.0}, Vector2{-1.0, 1.0}});
  world.add_obstacle(sim::Obstacle{Vector2{2.0, 0.0}, 0.5});
  auto a = std::make_shared<sim::Agent>(
      0.1, std::make_shared<DummyBehavior>(), std::make_shared<OmnidirectionalKinematics>(1.0),
      std::make_shared<sim::WaypointsTask>(sim::Waypoints{{1, 0}}, false, 0.1),
      nullptr, 0.1);
  world.add_agent(std::move(a));
  std::cout << "\nLoaded world\n====================" << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  world.run(100, (ng_float_t)0.1);
  std::cout
      << "\nAfter simulating for 10 s at 0.1 s steps\n===================="
      << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
