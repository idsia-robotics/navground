#include "navground/core/schema.h"
#include "navground/core/version.h"
#include "navground/core/yaml/schema_core.h"

namespace core = navground::core;

int main(int argc, char *argv[]) {
  return core::SchemaCommand("schema",
                             navground::core::build_info().get_version_string(),
                             "core")
      .run(argc, argv);
}