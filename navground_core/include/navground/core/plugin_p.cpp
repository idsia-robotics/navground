#include "navground/core/build_info.h"
#include "navground/core/version.h"

extern "C" {
navground::core::BuildDependencies plugin_build_dependencies() {
  return {{"core",
           {navground::core::build_info(), navground::core::get_build_info()}}};
}
}
