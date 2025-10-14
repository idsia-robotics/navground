#include "navground/sim/probe.h"

#include "navground/sim/experimental_run.h"

namespace navground::sim {

void RecordProbe::prepare(ExperimentalRun *run) {
  get_data()->set_item_shape(get_shape(*(run->get_world())));
}

void GroupRecordProbe::prepare(ExperimentalRun *run) {
  const bool use_uid = run->get_record_config().use_agent_uid_as_key;
  for (auto &[key, shape] : get_shapes(*(run->get_world()), use_uid)) {
    get_data(key)->set_item_shape(shape);
  }
}

void SensingProbe::init_ds(ExperimentalRun *run, unsigned agent_key,
                           const std::string &key, const core::Buffer &buffer) {
  auto ds = run->add_record(key, _name + "/" + std::to_string(agent_key));
  ds->config_to_hold_buffer(buffer);
  _data[agent_key][key] = ds;
}

void SensingProbe::prepare(ExperimentalRun *run) {
  const auto world = run->get_world();
  const bool use_uid = run->get_record_config().use_agent_uid_as_key;
  const auto agents = world->get_agents();
  if (_agent_indices.size() == 0) {
    for (unsigned i = 0; i < agents.size(); ++i) {
      _agent_indices.push_back(i);
    }
  } else {
    _agent_indices.erase(std::remove_if(_agent_indices.begin(),
                                        _agent_indices.end(),
                                        [&agents](const unsigned &i) {
                                          return i >= agents.size();
                                        }),
                         _agent_indices.end());
  }
  for (auto i : _agent_indices) {
    const auto agent = agents[i];
    auto state = get_state(agent.get());
    if (state) {
      if (_sensor) {
        _sensor->prepare_state(*state);
      }
      const unsigned agent_key = use_uid ? agent->uid : i;
      for (const auto &[key, buffer] : state->get_buffers()) {
        init_ds(run, agent_key, key, buffer);
      }
    }
  }
}

core::SensingState *SensingProbe::get_state(const Agent *agent) {
  if (_sensor) {
    return &(_states[agent->uid]);
  }
  auto behavior = agent->get_behavior();
  if (behavior) {
    return dynamic_cast<core::SensingState *>(
        behavior->get_environment_state());
  }
  return nullptr;
}

void SensingProbe::update(ExperimentalRun *run) {
  const auto world = run->get_world();
  const auto agents = world->get_agents();
  const bool use_uid = run->get_record_config().use_agent_uid_as_key;
  for (auto i : _agent_indices) {
    const auto agent = agents[i];
    auto state = get_state(agent.get());
    if (state) {
      if (_sensor) {
        _sensor->update(agent.get(), world.get(), state);
      }
      const unsigned agent_key = use_uid ? agent->uid : i;
      for (const auto &[key, buffer] : state->get_buffers()) {
        if (!_data[agent_key].count(key)) {
          std::cerr << key << " has not been initialized" << std::endl;
          init_ds(run, agent_key, key, buffer);
        }
        _data[agent_key][key]->append(buffer);
      }
    }
  }
}

} // namespace navground::sim