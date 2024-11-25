#include "navground/core/build_info.h"
#include "navground/core/version.h"
#include "navground/sim/build_info.h"
#include "navground/sim/version.h"

extern "C" {
#if defined(WIN32) || defined(_WIN32) ||                                       \
    defined(__WIN32) && !defined(__CYGWIN__)
__declspec(dllexport)
#else
__attribute__((visibility("default")))
#endif
void plugin_build_dependencies(void * arg) {
  auto bd = reinterpret_cast<navground::core::BuildDependencies *>(arg);
  bd->emplace("core", std::array<navground::core::BuildInfo, 2>{
                          navground::core::build_info(),
                          navground::core::get_build_info()});
  bd->emplace("sim", std::array<navground::core::BuildInfo, 2>{
                         navground::sim::build_info(),
                         navground::sim::get_build_info()});
}
}
