#ifndef NAVGROUND_CORE_BUILD_INFO_H
#define NAVGROUND_CORE_BUILD_INFO_H

#include "navground/core/types.h"
#include "navground/core/utilities.h"
#include <string>

#ifndef GIT_VERSION
#define GIT_VERSION "unknown"
#endif
#ifndef BUILD_STAMP
#define BUILD_STAMP "unknown"
#endif

/**
 * @brief     Stores build-time information
 */
struct BuildInfo {
  /**
   * The git commit 
   */
  std::string git_commit;
  /**
   * The build time UTC (%Y-%m-%dT%H:%M:%SZ).
   */
  std::string date;
  /**
   * Which type is used for floating-point numbers 
   * (configured by ``NAVGROUND_USES_DOUBLE``)
   */
  std::string floating_point_type;
  BuildInfo()
      : git_commit(GIT_VERSION), date(BUILD_STAMP),
        floating_point_type(get_type_name<ng_float_t>()) {}
};

#endif // NAVGROUND_CORE_BUILD_INFO_H
