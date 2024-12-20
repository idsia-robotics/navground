/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenarios/antipodal.h"
#include "navground/core/yaml/schema.h"
#include "navground/sim/sampling/sampler.h"

namespace navground::sim {

void AntipodalScenario::init_world(World *world,
                                   [[maybe_unused]] std::optional<int> seed) {
  Scenario::init_world(world, seed);
  auto agents = world->get_agents();
  const unsigned n = static_cast<unsigned>(agents.size());
  const ng_float_t da = (n < 1) ? 0 : 2 * core::PI / n;
  ng_float_t a = 0;
  NormalSampler<ng_float_t> x(0.0, position_noise);
  NormalSampler<ng_float_t> o(0.0, orientation_noise);
  RandomGenerator &rg = world->get_random_generator();
  if (shuffle) {
    std::shuffle(std::begin(agents), std::end(agents), rg);
  }
  for (auto &agent : agents) {
    const core::Vector2 p = radius * core::unit(a);
    agent->pose = Pose2(p, a + core::PI);
    if (position_noise) {
      agent->pose.position += core::Vector2{x.sample(rg), x.sample(rg)};
    }
    if (orientation_noise) {
      agent->pose.orientation += o.sample(rg);
    }
    // TODO(Jerome): maybe better to add noise to the target and/or the target
    // to -position
    agent->set_task(
        std::make_shared<WaypointsTask>(Waypoints{-p}, false, tolerance));
    a += da;
  }
}

const std::string AntipodalScenario::type = register_type<AntipodalScenario>(
    "Antipodal",
    {{"radius",
      core::Property::make(&AntipodalScenario::get_radius,
                           &AntipodalScenario::set_radius, default_radius,
                           "Radius of the circle", &YAML::schema::positive)},
     {"tolerance",
      core::Property::make(&AntipodalScenario::get_tolerance,
                           &AntipodalScenario::set_tolerance, default_tolerance,
                           "Goal tolerance", &YAML::schema::strict_positive)},
     {"position_noise",
      core::Property::make(
          &AntipodalScenario::get_position_noise,
          &AntipodalScenario::set_position_noise, default_position_noise,
          "Noise added to the initial position", &YAML::schema::positive)},
     {"orientation_noise",
      core::Property::make(
          &AntipodalScenario::get_orientation_noise,
          &AntipodalScenario::set_orientation_noise, default_orientation_noise,
          "Noise added to the initial orientation", &YAML::schema::positive)},
     {"shuffle",
      core::Property::make(
          &AntipodalScenario::get_shuffle, &AntipodalScenario::set_shuffle,
          default_shuffle,
          "Whether to shuffle the agents before initializing them")}});

} // namespace navground::sim
