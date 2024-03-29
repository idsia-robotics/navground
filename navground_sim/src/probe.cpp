#include "navground/sim/probe.h"

#include "navground/sim/experimental_run.h"

namespace navground::sim {

void RecordProbe::prepare(ExperimentalRun* run) {
  data->set_item_shape(get_shape(*(run->get_world())));
}

void GroupRecordProbe::prepare(ExperimentalRun* run) {
  for (auto& [key, shape] : get_shapes(*(run->get_world()))) {
    get_data(key)->set_item_shape(shape);
  }
}

}  // namespace navground::sim