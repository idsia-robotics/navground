#ifndef NAVGROUND_CORE_YAML_YAML_H
#define NAVGROUND_CORE_YAML_YAML_H

#include <iostream>
#include <memory>
#include <string>

#include "yaml-cpp/yaml.h"

namespace YAML {

/**
 * @brief      Load an object from a YAML node
 *
 * @param[in]  node  The YAML node
 *
 * @tparam     T     The type of the object
 *
 * @return     A shared pointer with the loaded object or ``nullptr`` if loading
 * fails.
 */
template <typename T>
inline std::shared_ptr<T> load_node(const Node &node) {
  return node.as<std::shared_ptr<T>>();
}

/**
 * @brief      Load an object from a YAML string
 *
 * @param[in]  value  The YAML string
 *
 * @tparam     T     The type of the object
 *
 * @return     A shared pointer with the loaded object or ``nullptr`` if loading
 * fails.
 */
template <typename T>
inline std::shared_ptr<T> load_string(const std::string &value) {
  YAML::Node node;
  try {
    node = YAML::Load(value);
  } catch (const YAML::ParserException &e) {
    std::cerr << e.what() << std::endl;
    return nullptr;
  }
  return load_node<T>(node);
}

/**
 * @brief      Dump an object to a YAML-string
 *
 * @param[in]  object   The object
 *
 * @tparam     T     The type of the object
 *
 * @return     The YAML string (empty if the pointer is null)
 */
template <typename T>
std::string dump(const T *object) {
  if (!object) return "";
  YAML::Emitter out;
  // TODO(?): these are not working
  out.SetFloatPrecision(6);
  out.SetDoublePrecision(6);
  // out << YAML::Precision(6) << YAML::Node(*object);
  out << YAML::Node(*object);
  return std::string(out.c_str());
}

}  // namespace YAML

#endif  // NAVGROUND_CORE_YAML_YAML_H
