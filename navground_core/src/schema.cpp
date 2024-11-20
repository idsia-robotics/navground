#include "navground/core/schema.h"
#include "navground/core/yaml/schema_core.h"

namespace core = navground::core;

int main(int argc, char *argv[]) {
  return core::SchemaCommand(
             "schema", "core",
             core::SchemaCommand::Schemas{
                 {"core", &YAML::schema::core},
                 {"behavior", &YAML::schema::base_with_ref<core::Behavior>},
                 {"behavior_modulation",
                  &YAML::schema::base_with_ref<core::BehaviorModulation>},
                 {"kinematics", &YAML::schema::base_with_ref<core::Kinematics>},
                 {"behavior_register",
                  &YAML::schema::registered<core::Behavior>},
                 {"behavior_modulation_register",
                  &YAML::schema::registered<core::BehaviorModulation>},
                 {"kinematics_register",
                  &YAML::schema::registered<core::Kinematics>},
             })
      .run(argc, argv);
}