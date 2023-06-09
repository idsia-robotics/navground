#ifndef NAVGROUND_CORE_REGISTER_H
#define NAVGROUND_CORE_REGISTER_H

#include <functional>
#include <memory>
#include <vector>

#include "./property.h"
#include "./utilities.h"
#include "navground_core_export.h"

namespace navground::core {

/**
 * @brief      Contains a register of sub-classes of ``T``, registered by name
 * using \ref register_type. ``T`` must also be a sub-class of \ref
 * navground::core::HasProperties or, at least, expose the static field
 * ``T::properties`` as properties are also registered and can be queried using
 * \ref type_properties.
 *
 * @tparam     T     The base class of all to be registered sub-classes.
 */
template <typename T>
struct NAVGROUND_CORE_EXPORT HasRegister {
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
  static std::map<std::string, Factory> &factory() {
    static std::map<std::string, Factory> f;
    return f;
  };

  /**
   * @brief      The registered properties
   *
   * @return     A map with the list of properties for all registered
   * sub-classes ``name -> properties``.
   */
  static std::map<std::string, Properties> &type_properties() {
    static std::map<std::string, Properties> p;
    return p;
  };

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
   *
   * @tparam     S     The type of the sub-class
   *
   * @return     The associated name.
   */
  template <typename S>
  static std::string register_type(const std::string &type) {
    // std::cout << "register_type " << get_type_name<S>() << " as " << type <<
    // std::endl;
    static_assert(std::is_base_of_v<T, S>);
    if (factory().count(type)) {
      // std::cerr << "Type " << type << " already registered for "
      //           << get_type_name<S>() << std::endl;
    } else {
      factory()[type] = []() { return std::make_shared<S>(); };
      type_properties()[type] = S::properties;
    }

    return type;
  }

  /**
   * @brief      Gets the name associated to the type of an object.
   *
   * @return     The associated name
   */
  virtual std::string get_type() const { return type; }

  /**
   * The name associated to this type.
   */
  static inline const std::string type = "";
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_REGISTER_H
