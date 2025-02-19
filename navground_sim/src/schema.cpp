#include "navground/core/schema.h"
#include "navground/sim/schema.h"
#include "navground/sim/version.h"

using navground::core::SchemaCommand;
using navground::sim::schemas;

int main(int argc, char *argv[]) {
  return SchemaCommand("schema",
                       navground::sim::build_info().get_version_string(), "sim",
                       schemas())
      .run(argc, argv);
}