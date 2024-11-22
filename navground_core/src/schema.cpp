#include "navground/core/schema.h"
#include "navground/core/yaml/schema_core.h"

namespace core = navground::core;

int main(int argc, char *argv[]) {
  return core::SchemaCommand("schema", "core").run(argc, argv);
}