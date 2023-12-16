#include "navground/sim/sampling/sampler.h"

namespace navground::sim {

static unsigned _seed = 0;

static RandomGenerator _generator(_seed);

RandomGenerator& random_generator() {
  // std::default_random_engine & random_generator() {
  return _generator;
}

void set_random_seed(unsigned seed) {
  _seed = seed;
  _generator.seed(seed);
}

unsigned get_random_seed() { return _seed; }

}  // namespace navground::sim
