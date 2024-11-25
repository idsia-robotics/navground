#ifndef NAVGROUND_SIM_BUILD_INFO_H
#define NAVGROUND_SIM_BUILD_INFO_H

#include "navground/core/build_info.h"
#include "navground/sim/export.h"

namespace navground::sim {

NAVGROUND_SIM_EXPORT core::BuildInfo get_build_info();
NAVGROUND_SIM_EXPORT core::BuildDependencies get_build_dependencies();

} // namespace navground::sim

#endif // NAVGROUND_SIM_BUILD_INFO_H
