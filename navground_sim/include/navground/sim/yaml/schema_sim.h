#ifndef NAVGROUND_SIM_YAML_SCHEMA_SIM_H
#define NAVGROUND_SIM_YAML_SCHEMA_SIM_H

#include "navground/core/yaml/schema_core.h"
#include "navground/sim/yaml/experiment.h"
#include "navground/sim/yaml/sampling.h"
#include "navground/sim/yaml/scenario.h"
#include "navground/sim/yaml/schema.h"
#include "yaml-cpp/yaml.h"

namespace navground::sim {

inline YAML::Node bundle_schema() {
  using namespace YAML::schema;

  auto node = core::bundle_schema();
  node["$id"] = id_name("sim");
  node["$defs"]["scenario"] = schema<Scenario>(true);
  node["$defs"]["scenario_register"] = register_schema<Scenario>();
  node["$defs"]["state_estimation"] = schema<StateEstimation>(true);
  node["$defs"]["state_estimation_register"] =
      register_schema<StateEstimation>();
  node["$defs"]["task"] = schema<Task>(true);
  node["$defs"]["task_register"] = register_schema<Task>();
  node["$defs"]["agent"] = schema<Agent>();
  node["$defs"]["wall"] = schema<Wall>();
  node["$defs"]["obstacle"] = schema<Obstacle>();
  node["$defs"]["world"] = schema<World>();
  node["$defs"]["behavior_sampler"] = sampler_schema<BehaviorSampler<>>(true);
  node["$defs"]["behavior_sampler_register"] =
      register_sampler_schema<BehaviorSampler<>>();
  node["$defs"]["behavior_modulation_sampler"] =
      sampler_schema<BehaviorModulationSampler<>>(true);
  node["$defs"]["behavior_modulation_sampler_register"] =
      register_sampler_schema<BehaviorModulationSampler<>>();
  node["$defs"]["kinematics_sampler"] =
      sampler_schema<KinematicsSampler<>>(true);
  node["$defs"]["kinematics_sampler_register"] =
      register_sampler_schema<KinematicsSampler<>>();
  node["$defs"]["state_estimation_sampler"] =
      sampler_schema<StateEstimationSampler<>>(true);
  node["$defs"]["state_estimation_sampler_register"] =
      register_sampler_schema<StateEstimationSampler<>>();
  node["$defs"]["task_sampler"] = sampler_schema<TaskSampler<>>(true);
  node["$defs"]["task_sampler_register"] =
      register_sampler_schema<TaskSampler<>>();
  node["$defs"]["group"] = schema<AgentSampler<>>();
  node["$defs"]["record_neighbors_config"] = schema<RecordNeighborsConfig>();
  node["$defs"]["record_sensing_config"] = schema<RecordSensingConfig>();
  node["$defs"]["experiment"] = schema<Experiment>();
  node["$defs"]["sampler"] = schema<GenericSampler>();
  node["$defs"]["boolean_sampler"] = schema<BooleanSampler>();
  node["$defs"]["number_sampler"] = schema<NumberSampler>();
  node["$defs"]["vector_sampler"] = schema<VectorSampler>();
  node["$defs"]["const"] = schema<ConstantSampler<void>>();
  node["$defs"]["sequence"] = schema<SequenceSampler<void>>();
  node["$defs"]["choice"] = schema<ChoiceSampler<void>>();
  node["$defs"]["grid"] = schema<GridSampler>();
  node["$defs"]["normal2d"] = schema<NormalSampler2D>();
  node["$defs"]["regular"] = schema<RegularSampler<void>>();
  node["$defs"]["uniform"] = schema<UniformSampler<void>>();
  node["$defs"]["normal"] = schema<NormalSampler<void>>();
  node["$defs"]["binary"] = schema<BinarySampler<void>>();
  return node;
}

} // namespace navground::sim

#endif // NAVGROUND_CORE_YAML_SCHEMA_H
