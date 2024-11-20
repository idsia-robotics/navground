#ifndef NAVGROUND_SIM_YAML_SCHEMA_SIM_H
#define NAVGROUND_SIM_YAML_SCHEMA_SIM_H

#include "navground/core/yaml/schema.h"
#include "navground/core/yaml/schema_core.h"
#include "navground/sim/experiment.h"
#include "navground/sim/sampling/agent.h"
#include "navground/sim/sampling/register.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/experiment.h"
#include "navground/sim/yaml/scenario.h"
#include "navground/sim/yaml/schema.h"
#include "yaml-cpp/yaml.h"

namespace YAML {
namespace schema {

inline Node sim() {
  using namespace navground::sim;

  Node node = core();
  node["$id"] = id_name("sim");
  node["$defs"]["state_estimation"] = base<StateEstimation>(true);
  node["$defs"]["state_estimation_register"] = registered<StateEstimation>();
  node["$defs"]["task"] = base<Task>(true);
  node["$defs"]["task_register"] = registered<Task>();
  node["$defs"]["agent"] = schema<Agent>();
  node["$defs"]["wall"] = schema<Wall>();
  node["$defs"]["obstacle"] = schema<Obstacle>();
  node["$defs"]["world"] = schema<World>();
  node["$defs"]["behavior_sampler"] = base<BehaviorSampler<>>(true);
  node["$defs"]["behavior_sampler_register"] =
      registered_sampler<BehaviorSampler<>>();
  node["$defs"]["behavior_modulation_sampler"] =
      base<BehaviorModulationSampler<>>(true);
  node["$defs"]["behavior_modulation_sampler_register"] =
      registered_sampler<BehaviorModulationSampler<>>();
  node["$defs"]["kinematics_sampler"] = base<KinematicsSampler<>>(true);
  node["$defs"]["kinematics_sampler_register"] =
      registered_sampler<KinematicsSampler<>>();
  node["$defs"]["state_estimation_sampler"] =
      base<StateEstimationSampler<>>(true);
  node["$defs"]["state_estimation_sampler_register"] =
      registered_sampler<StateEstimationSampler<>>();
  node["$defs"]["task_sampler"] = base<TaskSampler<>>(true);
  node["$defs"]["task_sampler_register"] = registered_sampler<TaskSampler<>>();
  node["$defs"]["group"] = schema<AgentSampler<>>();
  node["$defs"]["scenario"] = base<Scenario>(true);
  node["$defs"]["scenario_register"] = registered_sampler<Scenario>();
  node["$defs"]["record_neighbors_config"] = schema<RecordNeighborsConfig>();
  node["$defs"]["record_sensing_config"] = schema<RecordSensingConfig>();
  node["$defs"]["experiment"] = schema<Experiment>();
  node["$defs"]["integer_sampler"] = schema<Sampler<int>>();
  node["$defs"]["number_sampler"] = schema<Sampler<ng_float_t>>();
  node["$defs"]["boolean_sampler"] = schema<Sampler<bool>>();
  node["$defs"]["string_sampler"] = schema<Sampler<std::string>>();
  node["$defs"]["vector2_sampler"] =
      schema<Sampler<navground::core::Vector2>>();
  node["$defs"]["integer_array_sampler"] = schema<Sampler<std::vector<int>>>();
  node["$defs"]["number_array_sampler"] =
      schema<Sampler<std::vector<ng_float_t>>>();
  node["$defs"]["boolean_array_sampler"] = schema<Sampler<std::vector<bool>>>();
  node["$defs"]["string_array_sampler"] =
      schema<Sampler<std::vector<std::string>>>();
  node["$defs"]["vector2_array_sampler"] =
      schema<Sampler<std::vector<navground::core::Vector2>>>();
  node["$defs"]["const_sampler"] = schema<ConstantSampler<int>>();
  node["$defs"]["sequence_sampler"] = schema<SequenceSampler<void>>();
  node["$defs"]["choice_sampler"] = schema<ChoiceSampler<void>>();
  node["$defs"]["grid_sampler"] = schema<GridSampler>();
  node["$defs"]["regular_sampler"] = schema<RegularSampler<void>>();
  node["$defs"]["uniform_sampler"] = schema<UniformSampler<void>>();
  node["$defs"]["normal_sampler"] = schema<NormalSampler<void>>();
  return node;
}

} // namespace schema

} // namespace YAML

#endif // NAVGROUND_CORE_YAML_SCHEMA_H
