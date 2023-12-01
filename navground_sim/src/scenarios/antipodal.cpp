/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenarios/antipodal.h"
#include "navground/sim/sampling/sampler.h"


namespace navground::sim {

void AntipodalScenario::init_world(World *world, [[maybe_unused]] std::optional<int> seed) {
  Scenario::init_world(world, seed);
  auto agents = world->get_agents();
  const unsigned n = agents.size();
  const float da = (n < 1) ? 0.0f : 2 * M_PI / n;
  float a = 0.0f;
  NormalSampler<float> x(0.0, position_noise);
  NormalSampler<float> o(0.0, orientation_noise);
  if (shuffle) {
    std::shuffle(std::begin(agents), std::end(agents), random_generator());
  }
  for (auto &agent : agents) {
    const Vector2 p{radius * std::cos(a), radius * std::sin(a)};
    agent->pose = Pose2(p, a + M_PI);
    if (position_noise) {
      agent->pose.position += Vector2{x.sample(), x.sample()};
    }
    if (orientation_noise) {
      agent->pose.orientation += o.sample();
    }
    // TODO(Jerome): maybe better to add noise to the target and/or the target to -position
    agent->set_task(
        std::make_shared<WaypointsTask>(Waypoints{-p}, false, tolerance));
    a += da;
  }
}

}  // namespace navground::sim
