#include "navground/core/build_info.h"
#include "navground/core/version.h"
#include <cstdio>
#include <ctime>
#include <iostream>
#include <time.h>

namespace navground::core {

// parse a string  like DD.DD.DD-?.* to {DD, DD, DD}
// filling with zeros non parsable items.
static BuildInfo::Version parse_version(const std::string &value) {
  std::array<unsigned, 3> vs{0, 0, 0};
  sscanf(value.c_str(),"%u.%u.%u", &vs[0], &vs[1], &vs[2]);
  return vs;
}

std::string BuildInfo::get_version_string() const {
  std::string s;
  bool first = true;
  for (auto i : version) {
    if (!first) {
      s += ".";
    }
    first = false;
    s += std::to_string(i);
  }
  return s;
}

// Windows misses strptime.

#if defined(WIN32) || defined(_WIN32) ||                                       \
    defined(__WIN32) && !defined(__CYGWIN__)

static BuildInfo::Date parse_date(const std::string &value) {
  struct std::tm time_info{0};
  int y, m, d, H, M, S;
  int r = sscanf(value.c_str(), "%d-%d-%dT%d:%d:%dZ", &y, &m, &d, &H, &M, &S);
  if (r == 6) {
    time_info.tm_year = y - 1900;
    time_info.tm_mon = m - 1;
    time_info.tm_mday = d;
    time_info.tm_hour = H;
    time_info.tm_min = M;
    time_info.tm_sec = S;
  }
  time_info.tm_isdst = -1;
  return std::chrono::system_clock::from_time_t(_mkgmtime(&time_info));
}

#else

static BuildInfo::Date parse_date(const std::string &value) {
  struct std::tm time_info;
  if (!strptime(value.c_str(), BuildInfo::date_format, &time_info)) {
    return BuildInfo::Date();
  }
  time_info.tm_isdst = -1;
  return std::chrono::system_clock::from_time_t(timegm(&time_info)
                                                // std::mktime(&time_info)
  );
}

#endif

std::string BuildInfo::get_date_string() const {
  char buffer[80];
  const auto t = std::chrono::system_clock::to_time_t(date);
  std::strftime(buffer, 80, BuildInfo::local_date_format, localtime(&t));
  return std::string(buffer);
}

BuildInfo::BuildInfo(const std::string &git_describe, const std::string &date)
    : BuildInfo(parse_version(git_describe), git_describe, parse_date(date),
                std::string(get_type_name<ng_float_t>())) {};

BuildInfo get_build_info() { return build_info(); }
BuildDependencies get_build_dependencies() { return {}; }

std::string BuildInfo::to_string() const {
  std::string s;
  const std::string v = get_version_string();
  s = get_version_string() + "|" + floating_point_type + " (";
  if (git != v) {
    s += git + " ";
  }
  s += get_date_string() + ")";
  return s;
}

std::string BuildInfo::to_string_diff(const BuildInfo &other) const {
  const std::string v = get_version_string();
  std::string s = v;
  if (other.version != version) {
    s += "[" + other.get_version_string() + "]";
  }
  s += "|" + floating_point_type;
  if (other.floating_point_type != floating_point_type) {
    s += "[" + other.floating_point_type + "]";
  }
  s += " (";
  if (git != v || git != other.git) {
    s += git;
    if (other.git != git) {
      s += "[" + other.git + "]";
    }
    s += " ";
  }
  s += get_date_string();
  if (other.date != date) {
    s += "[" + other.get_date_string() + "]";
  }
  s += ")";
  return s;
}

std::string build_infos_to_string(const std::array<BuildInfo, 2> &infos) {
  return infos[0].to_string_diff(infos[1]);
}

} // namespace navground::core
