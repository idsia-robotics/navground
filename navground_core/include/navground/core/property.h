#ifndef NAVGROUND_CORE_PROPERTY_H
#define NAVGROUND_CORE_PROPERTY_H

#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "./common.h"
#include "./utilities.h"
#include "navground/core/export.h"
#include "navground/core/types.h"
#include "yaml-cpp/yaml.h"

namespace navground::core {

template <typename T> inline std::string_view field_type_name();
template <> inline std::string_view field_type_name<int>() { return "int"; }
template <> inline std::string_view field_type_name<ng_float_t>() {
  return "float";
}
template <> inline std::string_view field_type_name<bool>() { return "bool"; }
template <> inline std::string_view field_type_name<std::string>() {
  return "str";
}
template <> inline std::string_view field_type_name<Vector2>() {
  return "vector";
}
template <> inline std::string_view field_type_name<std::vector<int>>() {
  return "[int]";
}
template <> inline std::string_view field_type_name<std::vector<ng_float_t>>() {
  return "[float]";
}
template <> inline std::string_view field_type_name<std::vector<bool>>() {
  return "[bool]";
}
template <>
inline std::string_view field_type_name<std::vector<std::string>>() {
  return "[str]";
}
template <> inline std::string_view field_type_name<std::vector<Vector2>>() {
  return "[vector]";
}

inline std::string
get_type_name_with_scalar(const std::string &scalar_type_name, bool is_vector) {
  if (is_vector) {
    return "[" + scalar_type_name + "]";
  }
  return scalar_type_name;
}

inline std::tuple<std::string, bool>
get_scalar_type_name(const std::string &type_name) {
  std::string scalar_type_name = type_name;
  bool is_vector = false;
  if (type_name.size() > 2 && type_name.front() == '[' &&
      type_name.back() == ']') {
    scalar_type_name = type_name.substr(1, type_name.size() - 2);
    is_vector = true;
  }
  return {scalar_type_name, is_vector};
}

struct HasProperties;

/**
 * A function that gets a value of type ``T`` from an object of type ``C``.
 */
template <typename T, class C> using TypedGetter = std::function<T(const C *)>;

/**
 * A function that sets a value of type ``T`` of an object of type ``C``.
 */
template <typename T, class C>
using TypedSetter = std::function<void(C *, const T &)>;

/**
 * @brief      This class defines a property (similar to Python built-in
 * properties), i.e., a pair of getter and setter. In addition, it holds (for
 * auto-documentation) a default value, a description, and the name of the
 * involved types. A property has a value of type \ref Field, which is one of
 * ``bool``, ``int``, ``float``, ``string``, \ref navground::core::Vector2 or a
 * collection (a vector in C++, a list in Python) thereof. Properties are used
 * to configure sub-classes of \ref navground::core::HasProperties when using
 * bindings like YAML or Python. Properties values are accessed
 * using the methods exposed by \ref navground::core::HasProperties.
 */
struct Property {

  /**
   * The type of the scalar values held by the property.
   */
  using Scalar = std::variant<bool, int, ng_float_t, std::string, Vector2>;

  /**
   * The type of the value held by the property.
   */
  using Field =
      std::variant<bool, int, ng_float_t, std::string, Vector2,
                   std::vector<bool>, std::vector<int>, std::vector<ng_float_t>,
                   std::vector<std::string>, std::vector<Vector2>>;

  static std::string_view friendly_type_name(const Field &field) {
    return std::visit(
        [](auto &&arg) {
          using T = std::decay_t<decltype(arg)>;
          return field_type_name<T>();
        },
        field);
  }

  /**
   * The type of the property value getter, i.e., a function that gets the
   * property value from the owner.
   */
  using Getter = std::function<Field(const HasProperties *)>;
  /**
   * The type of the property value setter, i.e., a function that sets the
   * property value of the owner.
   */
  using Setter = std::function<void(HasProperties *, const Field &)>;

  /**
   * A custom YAML schema modifier
   */
  using Schema = std::function<void(YAML::Node &)>;

  /**
   * The property value getter.
   */
  Getter getter;
  /**
   * The property value setter.
   */
  Setter setter;
  /**
   * The property default value, i.e., the value of the property when the object
   * is initialized.
   */
  Field default_value;
  /**
   * The name of the property value type, used for auto documentation.
   */
  std::string type_name;
  /**
   * A textual description of the property, used for auto documentation.
   */
  std::string description;
  /**
   * The name of the property-owner type.
   */
  std::string owner_type_name;

  /**
   * Alternative deprecated names for the property
   */
  std::vector<std::string> deprecated_names;

  /**
   * Whether the property is readonly
   */
  bool readonly;

  /**
   * A custom YAML schema for the property to add validity constrains
   */
  Schema schema;

  /**
   * @brief      Makes a property from a pair of typed setters and getters.
   * It erases the type of the property (``T``)
   * and of the property owner (``C``) from the interface, replacing typed by
   * generic getters and setters. The property type is effectively preserved
   * inside the getter and setter.
   *
   *
   * @param[in]  getter         A typed getter
   * @param[in]  setter         A typed setter
   * @param[in]  default_value  The property default value
   * @param[in]  description    The property description
   * @param[in]  deprecated_names A list of deprecated alias names
   *                              for this property.
   * @param[in]  schema         An optional schema
   *
   * @tparam     T              The type of the property
   * @tparam     C              The type of the property owner
   *
   * @return     A property
   */
  template <typename T, class C>
  static Property make(const TypedGetter<T, C> &getter,
                       const TypedSetter<T, C> &setter, const T &default_value,
                       const std::string &description = "",
                       const Schema &schema = nullptr,
                       const std::vector<std::string> &deprecated_names = {}) {
    Property property;
    property.schema = schema;
    property.description = description;
    property.default_value = default_value;
    // property.type_name = std::string(get_type_name<T>());
    property.type_name = std::string(friendly_type_name(default_value));
    property.deprecated_names = deprecated_names;
    property.owner_type_name = std::string(get_type_name<C>());
    property.getter = [getter](const HasProperties *obj) {
      // getters can not be null
      const auto C_obj = dynamic_cast<const C *>(obj);
      if (!C_obj)
        throw std::bad_cast();
      return getter(C_obj);
    };
    property.readonly = (setter == nullptr);
    property.setter = [setter](HasProperties *obj,
                               const Property::Field &value) {
      // setters can be nil
      if (!setter) {
        std::cerr << "cannot set readonly property" << std::endl;
        return;
      }
      auto C_obj = dynamic_cast<C *>(obj);
      if (!C_obj)
        return;
      std::visit(
          [&setter, &C_obj](auto &&arg) {
            using V = std::decay_t<decltype(arg)>;
            if constexpr (std::is_convertible<V, T>::value) {
              setter(C_obj, static_cast<T>(arg));
            }
          },
          value);
    };
    return property;
  }

  /**
   * @brief      Makes a readonly property with a getter.
   * It erases the type of the property (``T``)
   * and of the property owner (``C``) from the interface, replacing typed by
   * generic getter. The property type is effectively preserved
   * inside the getter. The property default is set to ``T{}``.
   *
   *
   * @param[in]  getter         A typed getter
   * @param[in]  description    The property description
   * @param[in]  deprecated_names A list of deprecated alias names
   *                              for this property.
   * @param[in]  schema         An optional schema
   *
   * @tparam     T              The type of the property
   * @tparam     C              The type of the property owner
   *
   * @return     A property
   */
  template <typename T, class C>
  static Property make(const TypedGetter<T, C> &getter,
                       const std::string &description = "",
                       const Schema &schema = nullptr,
                       const std::vector<std::string> &deprecated_names = {}) {
    return make(getter, static_cast<const TypedSetter<T, C> &>(nullptr), T{},
                description, schema, deprecated_names);
  }

  /**
   * @brief      Makes a readonly property from a class getter.
   * It erases the type of the property (``T``)
   * and of the property owner (``C``) from the interface, replacing typed by
   * generic getter. The property type is effectively preserved
   * inside the getter. The property default is set to ``T{}``.
   *
   *
   * @param[in]  getter         The getter
   * @param[in]  description    The property description
   * @param[in]  deprecated_names A list of deprecated alias names
   *                              for this property.
   * @param[in]  schema         An optional schema
   *
   * @tparam     T              The type of the property
   * @tparam     C              The type of the property owner
   *
   * @return     A property
   */
  template <typename T, class C>
  static core::Property
  make(T (C::*getter)() const, const std::string &description = "",
       const std::vector<std::string> &deprecated_names = {},
       const Schema &schema = nullptr) {
    return make(static_cast<const TypedGetter<T, C> &>(getter), description,
                schema, deprecated_names);
  }

  /**
   * @brief      Makes a property from a pair of class setters and getters.
   * It erases the type of the property (``T``)
   * and of the property owner (``C``) from the interface, replacing typed by
   * generic getters and setters. The property type is effectively preserved
   * inside the getter and setter.
   *
   *
   * @param[in]  getter         A getter
   * @param[in]  setter         A setter
   * @param[in]  default_value  The property default value
   * @param[in]  description    The property description
   * @param[in]  deprecated_names A list of deprecated alias names
   *                              for this property.
   * @param[in]  schema         An optional schema
   *
   * @tparam     T              The type of the property
   * @tparam     C              The type of the property owner
   * @tparam     U              The type manipulated by the getter and setter.
   *
   * @return     A property
   */
  template <typename T, class C, typename U>
  static Property make(U (C::*getter)() const, void (C::*setter)(const U &),
                       const T &default_value,
                       const std::string &description = "",
                       const Schema &schema = nullptr,
                       const std::vector<std::string> &deprecated_names = {}) {
    return make(static_cast<const TypedGetter<T, C> &>(getter),
                static_cast<const TypedSetter<T, C> &>(setter),
                static_cast<T>(default_value), description, schema,
                deprecated_names);
  }

  /**
   * @brief      Makes a property from a pair of class setters and getters.
   * It erases the type of the property (``T``)
   * and of the property owner (``C``) from the interface, replacing typed by
   * generic getters and setters. The property type is effectively preserved
   * inside the getter and setter.
   *
   *
   * @param[in]  getter         A getter
   * @param[in]  setter         A setter
   * @param[in]  default_value  The property default value
   * @param[in]  description    The property description
   * @param[in]  deprecated_names A list of deprecated alias names
   *                              for this property.
   * @param[in]  schema         An optional schema
   *
   * @tparam     T              The type of the property
   * @tparam     C              The type of the property owner
   * @tparam     U              The type manipulated by the getter and setter.
   *
   * @return     A property
   */
  template <typename T, class C, typename U>
  static Property
  make(U (C::*getter)() const, void (C::*setter)(U), const T &default_value,
       const std::string &description = "", const Schema &schema = nullptr,
       const std::vector<std::string> &deprecated_names = {}) {
    return make(static_cast<const TypedGetter<T, C> &>(getter),
                static_cast<const TypedSetter<T, C> &>(setter),
                static_cast<T>(default_value), description, schema,
                deprecated_names);
  }

  /**
   * @brief      Makes a property that gets and sets a class member.
   * It erases the type of the property (``T``)
   * and of the property owner (``C``) from the interface, replacing typed by
   * generic getters and setters. The property type is effectively preserved
   * inside the getter and setter.
   *
   *
   * @param[in]  param          A class member
   * @param[in]  default_value  The property default value
   * @param[in]  description    The property description
   * @param[in]  deprecated_names A list of deprecated alias names
   *                              for this property.
   * @param[in]  schema         An optional schema
   *
   * @tparam     T              The type of the property
   * @tparam     C              The type of the property owner
   * @tparam     U              The type of param
   *
   * @return     A property
   */
  template <typename T, class C, typename U>
  static Property
  make_readwrite(U C::*param, const T &default_value,
                 const std::string &description = "",
                 const Schema &schema = nullptr,
                 const std::vector<std::string> &deprecated_names = {}) {
    return make<T, C>(
        [param](const C *c) -> T { return c->*param; },
        [param](C *c, const T &value) -> void { c->*param = value; },
        static_cast<T>(default_value), description, schema, deprecated_names);
  }

  /**
   * @brief      Makes a readonly property that gets a class member.
   * It erases the type of the property (``T``)
   * and of the property owner (``C``) from the interface, replacing typed by
   * generic getter. The property type is effectively preserved
   * inside the getter. The property default is set to ``T{}``.
   *
   *
   * @param[in]  param          A class member
   * @param[in]  description    The property description
   * @param[in]  deprecated_names A list of deprecated alias names
   *                              for this property.
   * @param[in]  schema         An optional schema
   *
   * @tparam     T              The type of the property
   * @tparam     C              The type of the property owner
   * @tparam     U              The type of param
   *
   * @return     A property
   */
  template <typename T, class C, typename U>
  static Property
  make_readonly(U C::*param, const std::string &description = "",
                const Schema &schema = nullptr,
                const std::vector<std::string> &deprecated_names = {}) {
    return make<T, C>([param](const C *c) -> T { return c->*param; },
                      description, schema, deprecated_names);
  }

  /**
   * @brief      Makes a readonly property that gets a class member.
   * It erases the type of the property (``T``)
   * and of the property owner (``C``) from the interface, replacing typed by
   * generic getter. The property type is effectively preserved
   * inside the getter. The property default is set to ``T{}``.
   *
   *
   * @param[in]  param          A class member
   * @param[in]  description    The property description
   * @param[in]  deprecated_names A list of deprecated alias names
   *                              for this property.
   * @param[in]  schema         An optional schema
   *
   * @tparam     C              The type of the property owner
   * @tparam     U              The type of param and property
   *
   * @return     A property
   */
  template <class C, typename U>
  static Property
  make_readonly(U C::*param, const std::string &description = "",
                const Schema &schema = nullptr,
                const std::vector<std::string> &deprecated_names = {}) {
    return make_readonly<U, C, U>(param, description, schema, deprecated_names);
  }
};

/**
 * A type the holds a dictionary of named properties ``name -> property``
 */
using Properties = std::map<std::string, Property>;

inline Properties operator+(const Properties &p1, const Properties &p2) {
  std::map<std::string, Property> r = p1;
  for (const auto &[k, v] : p2) {
    r[k] = v;
    // r.emplace(std::make_tuple(k, v));
  }
  return r;
}

/**
 * @brief      This class defines set and get methods to access named
 * \ref Property "properties".
 */
struct NAVGROUND_CORE_EXPORT HasProperties {
  virtual ~HasProperties() = default;

  /**
   * All the properties associated with a owner type.
   */
  // static inline std::map<std::string, Property> properties = Properties{};

  /**
   * @brief      Gets all properties associated with an owner.
   *
   * @return     The properties.
   */
  virtual const Properties &get_properties() const = 0;

  /**
   * @brief      Set the value of a named property.
   * Fails silently if no property can be found or if the value has a
   * non-compatible type.
   *
   * @param[in]  name   The name of the property
   * @param[in]  value  The desired value for the property
   */
  void set(const std::string &name, const Property::Field &value) {
    const auto &properties = get_properties();
    if (properties.count(name)) {
      const Property &property = properties.at(name);
      if (property.setter) {
        property.setter(this, value);
      }
    }
  }

  /**
   * @brief      Set the value of a named property, similar to \ref set but
   * casting the value to V.
   *
   * @param[in]  name   The name of the property
   * @param[in]  value  The desired value for the property
   *
   * @tparam     V      The type of the desired value
   */
  template <typename V>
  void set_value(const std::string &name, const V &value) {
    const auto &properties = get_properties();
    if (properties.count(name)) {
      const Property &property = properties.at(name);
      if (!property.setter)
        return;
      std::visit(
          [&property, this, &value](auto &&arg) {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_convertible<V, T>::value) {
              // std::cout << get_type_name<V>() << "=>" << get_type_name<T>()
              //           << std::endl;
              property.setter(this, static_cast<T>(value));
            }
          },
          property.default_value);
    }
  }

  /**
   * @brief      Gets the value of the specified property.
   *
   * @param[in]  name  The name of the property
   *
   * @throws     std::runtime_error A runtime error if no property is found.
   *
   * @return     The value of the property
   */
  Property::Field get(const std::string &name) const {
    const auto &properties = get_properties();
    if (properties.count(name)) {
      const Property &property = properties.at(name);
      if (property.getter) {
        return property.getter(this);
      }
    }
    throw std::runtime_error("No property " + name);
  }
};

} // namespace navground::core

template <typename T>
inline std::ostream &operator<<(std::ostream &os,
                                const std::vector<T> &values) {
  os << "[";
  bool f = true;
  for (const auto &value : values) {
    if (!f)
      os << ", ";
    f = false;
    os << value;
  }
  os << "]";
  return os;
}

inline std::ostream &operator<<(std::ostream &os,
                                const navground::core::Vector2 &value) {
  os << "[" << value[0] << ", " << value[1] << "]";
  return os;
}

inline std::ostream &operator<<(std::ostream &os,
                                const navground::core::Property::Field &value) {
  os << std::boolalpha;
  std::visit([&os](auto &&arg) { os << arg; }, value);
  os << std::noboolalpha;
  return os;
}

#endif // NAVGROUND_CORE_PROPERTY_H
