#ifndef NAVGROUND_CORE_BUILD_INFO_H
#define NAVGROUND_CORE_BUILD_INFO_H

#include "navground/core/export.h"
#include "navground/core/utilities.h"
#include <array>
#include <chrono>
#include <map>
#include <string>
#include <type_traits>

namespace navground::core {

/**
 * @brief Stores build-time information
 */
struct NAVGROUND_CORE_EXPORT BuildInfo {
  using Version = std::array<unsigned, 3>;
  // Not UTC, as C++17 does not have a UTC clock like C++20 does
  using Date = std::chrono::system_clock::time_point;
  /**
   * The date format
   */
  static constexpr const char *date_format = "%Y-%m-%dT%H:%M:%SZ";
  static constexpr const char *local_date_format = "%Y-%m-%dT%H:%M";

  /**
   * The version (major.minor.patch)
   */
  Version version;
  /**
   * The git describe
   */
  std::string git;
#if 0
  /**
   * The build time UTC (%Y-%m-%dT%H:%M:%SZ).
   */
  std::string date;
#endif
  /**
   * The build time
   */
  Date date;
  /**
   * Which type is used for floating-point numbers
   * (configured by ``NAVGROUND_USES_DOUBLE``)
   */
  std::string floating_point_type;
  /**
   * @brief      Gets the version string.
   *
   * @return     The version string: ``"<mayor>.<minor>.<patch>"``
   */
  std::string get_version_string() const;
  /**
   * @brief      Gets the date string.
   *
   * @return     The date string.
   */
  std::string get_date_string() const;

  /**
   * @brief      Returns a string representation of the object.
   *
   * @return     String representation of the object.
   */
  std::string to_string() const;
  /**
   * @brief      Returns a string representation of the object.
   *
   * @return     String representation of the object.
   */
  std::string to_string_diff(const BuildInfo &other) const;
  /**
   * @brief      Construct an instance.
   *
   * @param[in]  version  The version ({major, minor, patch})
   * @param[in]  git  The output of ``git describe``
   * @param[in]  date The build date.
   * @param[in]  floating_point_type Which type is used for floating-point
   * numbers.
   */
  BuildInfo(const Version &version, const std::string &git, const Date &date,
            const std::string &floating_point_type)
      : version(version), git(git), date(date),
        floating_point_type(floating_point_type) {}
  /**
   * @brief      Construct an instance.
   *
   * @param[in]  git_describe  The output of ``git describe``
   * @param[in]  utc_date    The utc date in format \ref date_format.
   */
  BuildInfo(const std::string &git_describe, const std::string &utc_date);

  bool operator==(const BuildInfo &other) {
    return git == other.git && date == other.date &&
           floating_point_type == other.floating_point_type;
  }
};

/**
 * @brief      Holds the build information
 * at build-time and run-time of a dependency
 */
struct DependencyInfo {
  /**
   * The build info of the dependency at build time of this library
   */
  BuildInfo build;
  /**
   * The build info of the currently loaded dependency
   */
  BuildInfo run;

  /**
   * @brief      Represent the different between a pair of \ref BuildInfo
   * using  \ref  BuildInfo::to_string_diff.
   *
   * @return     ``build.to_string_diff(run)``
   */
  std::string to_string() const { return build.to_string_diff(run); }
};

/**
 * A map of dependencies: name -> {build-time version, run-time version}
 */
using BuildDependencies = std::map<std::string, DependencyInfo>;

/**
 * @brief      Gets the build information.
 *
 * @return     The build information.
 */
NAVGROUND_CORE_EXPORT BuildInfo get_build_info();
/**
 * @brief      Gets the build dependencies.
 *
 * @return     The build dependencies.
 */
NAVGROUND_CORE_EXPORT BuildDependencies get_build_dependencies();

} // namespace navground::core

#endif // NAVGROUND_CORE_BUILD_INFO_H
