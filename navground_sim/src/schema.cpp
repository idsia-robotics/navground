#include "navground/core/schema.h"
#include "navground/sim/schema.h"

using navground::core::SchemaCommand;
using navground::sim::schemas;

int main(int argc, char *argv[]) {
  return SchemaCommand("schema", "sim", schemas()).run(argc, argv);
}