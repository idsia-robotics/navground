#ifndef NAVGROUND_CORE_CWD_H
#define NAVGROUND_CORE_CWD_H

#include <filesystem>
#include <optional>

namespace navground::core {

class CurrentWorkingDirectory {
public:
  explicit CurrentWorkingDirectory(
      const std::optional<std::filesystem::path> & value)
      : previous(std::nullopt) {
    if (!value)
      return;
    previous = std::filesystem::current_path();
    if (std::filesystem::is_directory(*value)) {
      std::filesystem::current_path(*value);
    } else {
      std::filesystem::current_path(value->parent_path());
    }
  }
  ~CurrentWorkingDirectory() {
    if (previous) {
      std::filesystem::current_path(*previous);
    }
  }

private:
  std::optional<std::filesystem::path> previous;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_CWD_H