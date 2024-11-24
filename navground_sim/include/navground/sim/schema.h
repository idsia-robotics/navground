/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCHEMA_H
#define NAVGROUND_SIM_SCHEMA_H

#include "navground/core/schema.h"
#include "navground/sim/yaml/schema_sim.h"

namespace navground::sim {

using core::SchemaCommand;

/**
 * @brief      Returns the bundle json-schema for \ref navground::sim
 *
 * @return     A json-schema encoded as a \ref YAML::Node.
 */
inline SchemaCommand::Schemas schemas() {
  return {
      {"sim", [](const argparse::ArgumentParser &) { return bundle_schema(); }},
      {"scenario", &SchemaCommand::component<sim::Scenario>},
      {"state_estimation", &SchemaCommand::component<sim::StateEstimation>},
      {"task", &SchemaCommand::component<sim::Task>},
      {"agent", &SchemaCommand::schema<sim::Agent>},
      {"experiment", &SchemaCommand::schema<sim::World>},
      {"world", &SchemaCommand::schema<sim::World>}};
}

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCHEMA_H */
