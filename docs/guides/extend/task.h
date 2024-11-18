#include "navground/sim/task.h"

namespace sim = navground::sim;

struct MyTask : public sim::Task {
  // CAN override
  // executed at the start of the simulation
  // void prepare(sim::Agent *agent, sim::World *world) override;

  // CAN override
  // executed during the the simulation, should update the target
  // or call the controller
  void update(sim::Agent *agent, sim::World *world, ng_float_t time) override;

  // CAN override
  // return if we are done or not
  bool done() override;

  // Set the log size to a constant value
  static constexpr log_size = 0;

  // CAN override
  // return the size of the data with log for each event
  // do not override or return 0 if you are not logging anything
  // should return a constant
  unsigned get_log_size() const override { return log_size};
};