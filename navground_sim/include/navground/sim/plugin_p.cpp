#include "navground/core/build_info.h"
#include "navground/core/version.h"
#include "navground/sim/build_info.h"
#include "navground/sim/version.h"

extern "C" {
navground::core::BuildDependencies plugin_build_dependencies() {
  return {{"core",
           {navground::core::build_info(), navground::core::get_build_info()}},
          {"sim",
           {navground::sim::build_info(), navground::sim::get_build_info()}}};
}
}
