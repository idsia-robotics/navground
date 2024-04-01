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

/* NOTE about YAML serialization of floating points in YAML nodes

float and double are converted to string when building the node
in include/yaml-cpp/node/convert.h
#define YAML_DEFINE_CONVERT_STREAMABLE(type, negative_op)
with MAXIMAL PRECISION.

If we want to reduce the precision, we need to change it there!

    static Node encode(const type& rhs) {
      std::stringstream stream;
--->  stream.precision(std::numeric_limits<type>::max_digits10);
      conversion::inner_encode(rhs, stream);
      return Node(stream.str());
    }

or to convert all floats to strings as we like.
*/

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
  // out.SetFloatPrecision(6);
  // out.SetDoublePrecision(6);
  out << YAML::Node(*object);
  return std::string(out.c_str());
}

}  // namespace YAML

#endif  // NAVGROUND_CORE_YAML_YAML_H
