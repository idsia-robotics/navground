#ifndef NAVGROUND_CORE_ATTRIBUTE_H
#define NAVGROUND_CORE_ATTRIBUTE_H

#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "./common.h"
#include "./property.h"
#include "./utilities.h"
#include "navground/core/export.h"
#include "navground/core/types.h"

namespace navground::core {

using Attribute = Property::Field;

/**
 * A type the holds a dictionary of named attributes ``name -> value``
 */
using Attributes = std::map<std::string, Attribute>;

/**
 * @brief      This class defines set and get methods to access dynamic
 * attributes
 */

struct NAVGROUND_CORE_EXPORT HasAttributes {

  /**
   * @brief      Gets all attributes.
   *
   * @return     The attributes.
   */
  const Attributes &get_attributes() const { return _attributes; }

  /**
   * @brief      Sets all attributes.
   *
   * @param[in]  values The attributes.
   */
  void set_attributes(const Attributes &values) { _attributes = values; }

  /**
   * @brief      Set the value of a named attribute.
   *
   * @param[in]  name   The name of the property
   * @param[in]  value  The desired value for the attribute
   */
  void set(const std::string &name, const Property::Field &value) {
    _attributes[name] = value;
  }

  /**
   * @brief      Gets the value of the specified attribute.
   *
   * @param[in]  name  The name of the attribute
   *
   * @return     The value of the attribute
   */
  std::optional<Attribute> get(const std::string &name) const {
    if (_attributes.count(name)) {
      return _attributes.at(name);
    }
    return std::nullopt;
  }

  /**
   * @brief      Checks if the specified attribute is set.
   *
   * @param[in]  name  The name of the attribute
   *
   * @return     True if the attribute is present.
   */
  std::optional<Attribute> has(const std::string &name) const {
    return _attributes.count(name) > 0;
  }

  /**
   * @brief      Remove the specified attribute.
   *
   * @param[in]  name  The name of the attribute
   *
   */
  void remove(const std::string &name) { _attributes.erase(name); }

  /**
   * @brief      Remove all attributes
   */
  void clear() { _attributes.clear(); }

  /**
   * @brief      Gets the value of the specified attribute.
   *
   * @param[in]  name  The name
   *
   * @tparam     T     The desired type
   *
   * @return     The value static casted to ``T`` if possible, else null.
   */
  template <typename T>
  std::optional<T> get_typed(const std::string &name) const {
    if (_attributes.count(name)) {
      return std::visit(
          [](auto &&arg) {
            using V = std::decay_t<decltype(arg)>;
            if constexpr (std::is_convertible<V, T>::value) {
              return static_cast<T>(arg);
            }
            return std::nullopt;
          },
          _attributes.at(name));
    }
    return std::nullopt;
  }

protected:
  HasAttributes() : _attributes() {};

private:
  Attributes _attributes;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_ATTRIBUTE_H
