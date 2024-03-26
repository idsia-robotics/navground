#ifndef NAVGROUND_CORE_PLUGINS_H
#define NAVGROUND_CORE_PLUGINS_H

#include <filesystem>
#include <optional>
#include "navground_core_export.h"

namespace navground::core {

/**
 * @brief      Loads plugins
 *
 * Plugins are shared libraries that extend one or more registered classes.
 *
 * @param[in]  plugins    A list of paths separated by ";".
 * @param[in]  env        An environment variable with additional paths
 *                        separated by ";". If empty, it defaults to
 *                        ``"NAVGROUND_PLUGINS"``
 * @param[in]  directory  A directory with files containing additional paths
 *                        separated by ";". If null, it defaults to
 *                        the value of the macro ``NAVGROUND_PLUGINS_PATH``
 */
void NAVGROUND_CORE_EXPORT load_plugins(
    const std::string& plugins = "", std::string env = "",
    std::optional<std::filesystem::path> directory = std::nullopt);

}  // namespace navground::core

#endif  // NAVGROUND_CORE_PLUGINS_H
