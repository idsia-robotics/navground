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

void SensingProbe::prepare(ExperimentalRun* run) {
  const auto world = run->get_world();
  const bool use_uid = run->get_record_config().use_agent_uid_as_key;
  const auto agents = world->get_agents();
  if (_agent_indices.size() == 0) {
    for (unsigned i = 0; i < agents.size(); ++i) {
      _agent_indices.push_back(i);
    }
  } else {
    _agent_indices.erase(
        std::remove_if(
            _agent_indices.begin(), _agent_indices.end(),
            [&agents](const unsigned& i) { return i >= agents.size(); }),
        _agent_indices.end());
  }
  for (auto i : _agent_indices) {
    const auto agent = agents[i];
    auto& state = _states[agent->uid];
    _sensor->prepare(state);
    const unsigned agent_key = use_uid ? agent->uid : i;
    for (const auto& [key, buffer] : state.get_buffers()) {
      auto ds = run->add_record(key, std::to_string(agent_key) + "/" + _name);
      ds->config_to_hold_buffer(buffer);
      _data[agent_key][key] = ds;
    }
  }
}

void SensingProbe::update(ExperimentalRun* run) {
  const auto world = run->get_world();
  const auto agents = world->get_agents();
  const bool use_uid = run->get_record_config().use_agent_uid_as_key;
  for (auto i : _agent_indices) {
    const auto agent = agents[i];
    if (_states.count(agent->uid) == 0) continue;
    auto& state = _states[agent->uid];
    if (_sensor) {
      _sensor->update(agent.get(), world.get(), &state);
    }
    const unsigned agent_key = use_uid ? agent->uid : i;
    for (const auto& [key, buffer] : state.get_buffers()) {
      _data[agent_key][key]->append(buffer);
    }
  }
}

}  // namespace navground::sim