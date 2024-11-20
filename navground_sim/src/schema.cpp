#include "navground/core/schema.h"
#include "navground/core/yaml/schema_core.h"
#include "navground/sim/yaml/schema_sim.h"

namespace core = navground::core;
namespace sim = navground::sim;

int main(int argc, char *argv[]) {
  return core::SchemaCommand(
             "schema", "sim",
             core::SchemaCommand::Schemas{
                 {"core", &YAML::schema::core},
                 {"sim", &YAML::schema::sim},
                 {"behavior", &YAML::schema::base_with_ref<core::Behavior>},
                 {"modulation",
                  &YAML::schema::base_with_ref<core::BehaviorModulation>},
                 {"kinematics", &YAML::schema::base_with_ref<core::Kinematics>},
                 {"state_estimation",
                  &YAML::schema::base_with_ref<sim::StateEstimation>},
                 {"task", &YAML::schema::base_with_ref<sim::Task>},
                 {"agent", &YAML::schema::schema<sim::Agent>},
                 {"world", &YAML::schema::schema<sim::World>},
                 {"scenario", &YAML::schema::base_with_ref<sim::Scenario>},
                 {"experiment", &YAML::schema::schema<sim::World>},
                 {"behavior_register",
                  &YAML::schema::registered<core::Behavior>},
                 {"behavior_modulation_register",
                  &YAML::schema::registered<core::BehaviorModulation>},
                 {"kinematics_register",
                  &YAML::schema::registered<core::Kinematics>},
                 {"state_estimation_register",
                  &YAML::schema::registered<sim::StateEstimation>},
                 {"task_register", &YAML::schema::registered<sim::Task>},
                 {"scenario_register",
                  &YAML::schema::registered_sampler<sim::Scenario>},
             })
      .run(argc, argv);
}