#ifndef NAVGROUND_CORE_REGISTER_H
#define NAVGROUND_CORE_REGISTER_H

#include <functional>
#include <memory>
#include <string_view>
#include <typeindex>
#include <vector>

#include "navground/core/export.h"
#include "navground/core/property.h"
#include "navground/core/utilities.h"
#include "yaml-cpp/yaml.h"

namespace navground::core {

using PropertyRegister = std::map<std::string, Properties>;
using Schema = std::function<void(YAML::Node &)>;
using SchemaRegister = std::map<std::string, Schema>;

/**
 * @brief      Contains a register of sub-classes of ``T``, registered by name
 * using \ref register_type.
 *
 * @tparam     T     The base class of all to be registered sub-classes.
 */
template <typename T>
struct
#if defined(_MSC_VER)
    NAVGROUND_CORE_NO_EXPORT
#else
    NAVGROUND_CORE_EXPORT
#endif // _MSC_VER
        HasRegister : public HasProperties {
  /**
   * A shared pointer to an object of type ``T``.
   */
  using C = std::shared_ptr<T>;
  /**
   * Factory function to construct objects of type ``T``.
   */
  using Factory = std::function<C()>;

  /**
   * @brief      The registered factories
   *
   * @return     A map of registered factory functions ``name -> factory``.
   */
  static std::map<std::string, Factory> &factory();
  /**
   * @brief      The registered properties
   *
   * @return     A map with the list of properties for all registered
   * sub-classes ``name -> properties``.
   */
  static PropertyRegister &type_properties();
  /**
   * @brief      The registered properties
   *
   * @return     A map with the list of properties for all registered
   * sub-classes ``name -> properties``.
   */
  static SchemaRegister &type_schema();

  /**
   * @brief      Create a shared pointer of type ``T``, selecting the factory
   * method by name.
   *
   * @param[in]  type  The associated sub-class name.
   *
   * @return     A pointer to an object of a registered sub-class.
   * Returns ``nullptr`` (C++) or ``None``(Python) in case the desired name is
   * not found.
   */
  static C make_type(const std::string &type) {
    if (factory().count(type)) {
      return factory()[type]();
    }
    // std::cerr << "Type " << type << " is not a registered "
    //           << get_type_name<T>() << std::endl;
    return nullptr;
  }

  /**
   * @brief      Check whether a type name has been registered.
   *
   * @param[in]  type  The associated sub-class name.
   *
   * @return     True if the type name has been registered
   */
  static bool has_type(const std::string &type) {
    return factory().count(type);
  }

  /**
   * @brief      Reads the names of all registered sub-classes
   *
   * @return     A collection of names associated with registered sub-classes.
   */
  static std::vector<std::string> types() {
    std::vector<std::string> keys;
    std::transform(factory().begin(), factory().end(), back_inserter(keys),
                   [](std::pair<std::string, Factory> p) { return p.first; });
    return keys;
  }

  /**
   * @brief      Register a sub-class of ``T``, associating it with a
   * user-defined name.
   *
   * @param[in]  type  The user-defined name to be associated with the
   * sub-class.
   * @param[in]  properties Registered properties
   * @param[in]  schema An optional schema for custom YAML decoders.
   *
   * @tparam     S     The type of the sub-class
   *
   * @return     The associated name.
   */
  template <typename S>
  static std::string register_type(const std::string &type,
                                   const Properties &properties = {},
                                   const Schema &schema = nullptr) {
    // std::cout << "register_type " << get_type_name<S>() << " as " << type
    //           << std::endl;
    static_assert(std::is_base_of_v<T, S>);
    // if (factory().count(type)) {
    // std::cerr << "Type " << type << " already registered for "
    //           << get_type_name<S>() << std::endl;
    // } else {

    factory()[type] = []() { return std::make_shared<S>(); };
    type_properties()[type] = properties;
    type_names()[std::type_index(typeid(S))] = type;
    if (schema) {
      type_schema()[type] = schema;
    }
    // }

    return type;
  }

  static std::map<std::type_index, std::string> &type_names();

  /**
   * @brief      Gets the name associated to the type of an object in the
   * register.
   *
   * @return     The associated name or empty if not registered.
   */
  virtual std::string get_type() const {
    const auto &tn = type_names();
    const auto i = std::type_index(typeid(*this));
    if (tn.count(i)) {
      return tn.at(i);
    }
    return "";
  }

  /**
   * @brief      Gets name associated to the type in the register.
   *
   * @tparam     S     The type
   *
   * @return     The associated name or empty if not registered.
   */
  template <typename S> static std::string get_type() {
    const auto &tn = type_names();
    const auto i = std::type_index(typeid(S));
    if (tn.count(i)) {
      return tn.at(i);
    }
    return "";
  }

  inline static const Properties properties = {};

  /**
   * @brief      Gets the registered properties.
   *
   * @return     The properties or empty if not registered.
   */
  const Properties &get_properties() const override {
    const auto t = get_type();
    if (t.empty()) {
      return properties;
    }
    return type_properties().at(t);
  }

  /**
   * @brief      Gets the registered properties for this type.
   *
   * @tparam     S     The type
   *
   * @return     The properties or empty if not registered.
   */
  template <typename S> static const Properties &get_properties() {
    const auto t = get_type<S>();
    if (t.empty()) {
      return properties;
    }
    return type_properties().at(t);
  }

  /**
   * @brief      Allows to customize YAML encoding
   *
   * @param      node  The YAML node
   */
  virtual void encode([[maybe_unused]] YAML::Node &node) const {};
  /**
   * @brief      Allows to customize YAML decoding
   *
   * @param      node  The YAML node
   */
  virtual void decode([[maybe_unused]] const YAML::Node &node) {};
};

} // namespace navground::core

/**
 * @brief      Adds type declaration
 *
 * \code{.cpp}
 * static const std::string type;
 * \endcode
 */
#define DECLARE_TYPE                                                           \
  /** @private */                                                              \
  static const std::string type;

/**
 * @brief Adds properties declaration.
 *
 * \code{.cpp}
 * static const std::map<std::string, navground::core::Property> properties;
 * \endcode
 */
#define DECLARE_PROPERTIES                                                     \
  /** @private */                                                              \
  static const std::map<std::string, navground::core::Property> properties;

/**
 * @brief Combine \ref DECLARE_TYPE and \ref DECLARE_PROPERTIES
 *
 * \code{.cpp}
 * static const std::string type;
 * static const std::map<std::string, navground::core::Property> properties;
 * \endcode
 */
#define DECLARE_TYPE_AND_PROPERTIES                                            \
  static const std::string type;                                               \
  DECLARE_PROPERTIES

/**
 * @private
 */
#define DEFINE_REGISTERS(Class)                                                \
  template <> PropertyRegister &HasRegister<Class>::type_properties() {        \
    static PropertyRegister p;                                                 \
    return p;                                                                  \
  }                                                                           \
  template <> SchemaRegister &HasRegister<Class>::type_schema() {              \
    static SchemaRegister r;                                                   \
    return r;                                                                  \
  }                                                                           \
  template <>                                                                  \
  std::map<std::string, HasRegister<Class>::Factory> &                         \
  HasRegister<Class>::factory() {                                              \
    static std::map<std::string, HasRegister<Class>::Factory> f;               \
    return f;                                                                  \
  }                                                                           \
  template <>                                                                  \
  std::map<std::type_index, std::string> &HasRegister<Class>::type_names() {   \
    static std::map<std::type_index, std::string> _r;                          \
    return _r;                                                                 \
  }

#endif // NAVGROUND_CORE_REGISTER_H
