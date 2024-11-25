#include "navground/core/build_info.h"
#include "navground/core/version.h"
#include <ctime>
#include <time.h>

namespace navground::core {

// parse a string  like DD.DD.DD-?.* to {DD, DD, DD}
// filling with zeros non parsable items.
static BuildInfo::Version parse_version(const std::string &value) {
  std::array<unsigned, 3> vs{0, 0, 0};
  size_t a = 0;
  for (size_t i = 0; i < 2; i++) {
    auto b = value.find('.');
    if (b == std::string::npos) {
      return vs;
    } else {
      try {
        vs[i] = std::stoul(value.substr(a, b));
      } catch (...) {
        return vs;
      }
    }
    a = b + 1;
  }
  try {
    auto b = value.find('-');
    vs[2] = std::stoul(value.substr(a, b));
  } catch (...) {
  }
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

static BuildInfo::Date parse_date(const std::string &value) {
  struct std::tm time_info;
  if (!strptime(value.c_str(), BuildInfo::date_format, &time_info)) {
    return BuildInfo::Date();
  }
  time_info.tm_isdst = -1;
  return std::chrono::system_clock::from_time_t(
      // TODO(Jerome): WIN32
      timegm(&time_info)
      // std::mktime(&time_info)
  );
}

std::string BuildInfo::get_date_string() const {
  char buffer[80];
  struct std::tm time_info;
  const auto t = std::chrono::system_clock::to_time_t(date);
  std::strftime(buffer, 80, BuildInfo::local_date_format,
                localtime_r(&t, &time_info));
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
