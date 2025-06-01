#include "navground/core/echo.h"
#include "./echo.h"
#include "navground/sim/version.h"

int main(int argc, char *argv[]) {
  return navground::core::EchoCommand(
             "echo", navground::sim::build_info().get_version_string(), echos())
      .add_setup(sampler_setup)
      .run(argc, argv);
}