#include "navground/sim/build_info.h"
#include "navground/core/version.h"
#include "navground/sim/version.h"

namespace navground::sim {

core::BuildInfo get_build_info() { return build_info(); }
core::BuildDependencies get_build_dependencies() {
  return {{"core", {core::build_info(), core::get_build_info()}}};
}

} // namespace navground::sim
