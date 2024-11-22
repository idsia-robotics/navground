/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCHEMA_H
#define NAVGROUND_SIM_SCHEMA_H

#include "navground/core/schema.h"
#include "navground/core/yaml/schema_core.h"
#include "navground/sim/yaml/schema_sim.h"

namespace navground::sim {

using core::SchemaCommand;

inline SchemaCommand::Schemas schemas() {
  return {
      {"sim",
       [](const argparse::ArgumentParser &) { return YAML::schema::sim(); }},
      {"state_estimation", &SchemaCommand::component<sim::StateEstimation>},
      {"task", &SchemaCommand::component<sim::Task>},
      {"agent", &SchemaCommand::schema<sim::Agent>},
      {"world", &SchemaCommand::schema<sim::World>},
      {"scenario",
       [](const argparse::ArgumentParser &parser) {
         const std::string type = parser.get<std::string>("type");
         if (type.empty()) {
           return YAML::schema::base_with_ref<Scenario>();
         }
         return YAML::schema::sampler_schema_of_type<Scenario>(type);
       }},
      {"experiment", &SchemaCommand::schema<sim::World>},
      {"state_estimation_register",
       &SchemaCommand::components<sim::StateEstimation>},
      {"task_register", &SchemaCommand::components<sim::Task>},
      {"scenario_register", [](const argparse::ArgumentParser &) {
         return YAML::schema::registered_sampler<Scenario>();
       }}};
}

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCHEMA_H */
