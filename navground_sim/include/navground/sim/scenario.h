/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIO_H
#define NAVGROUND_SIM_SCENARIO_H

#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "navground/core/property.h"
#include "navground/core/register.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/export.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/world.h"

using navground::core::Disc;
using navground::core::HasProperties;
using navground::core::HasRegister;
using navground::core::LineSegment;

namespace navground::sim {

/**
 * @brief      A scenario is a generator of \ref World.
 *
 * Sub-classes should override \ref init_world with a custom initialization
 * that is performed each time a world is sampled (e.g., during an experiment).
 *
 * They can also be customized by adding groups using \ref add_group
 * and/or initializers using \ref add_init, although these are not exposed to
 * YAML.
 *
 */
struct NAVGROUND_SIM_EXPORT Scenario : virtual public HasRegister<Scenario> {

  using Type = Scenario;

  /**
   * @brief      A group of agents that can be generated and added to the world.
   */
  struct Group {
    /**
     * @brief      Generate and add agents to the world.
     *
     * @param      world  The world
     * @param      seed  An optional random seed
     */
    virtual void add_to_world(World *world,
                              std::optional<unsigned> seed = std::nullopt) = 0;
    virtual ~Group() = default;
  };

  /**
   * A world initializer: a function that takes as input world and optional seed
   * and perform some initialization on the world.
   */
  using Init = std::function<void(World *, std::optional<unsigned>)>;
  /**
   * A collection of world initializers
   */
  using Inits = std::map<std::string, Init>;

  /**
   * A collection of groups
   */
  using Groups = std::vector<std::shared_ptr<Group>>;

  /**
   * A collection of property samplers
   */
  using PropertySamplers =
      std::map<std::string, std::shared_ptr<PropertySampler>>;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  inits  The collection of world initializers to use.
   */
  explicit Scenario(const Inits &inits = {})
      : obstacles(), walls(), initializers(inits), groups(),
        property_samplers() {}

  /**
   * @brief      Initializes the world.
   *
   * Users can specialize this method to specialize a scenario but
   * should call \ref make_world when creating a world.
   *
   * @param      world The world
   * @param      seed  The random seed
   */
  virtual void init_world(World *world, std::optional<int> seed = std::nullopt);

  /**
   * @brief      Applies the initializers from \ref get_inits
   *
   * Should be called after \ref init_world to complete the initialization,
   * as \ref make_world does automatically.
   *
   * @param      world  The world
   */
  void apply_inits(World *world);

  /**
   * @brief      Creates and initialize a world.
   *
   * @param      seed  The random seed
   */
  std::shared_ptr<World> make_world(std::optional<int> seed = std::nullopt);

  /**
   * @brief      Adds a world initializer.
   *
   * @param[in]  initializer  The initializer
   *
   * @return     The associated key
   */
  std::string add_init(const Init &initializer) {
    const auto key = key_for_next_init();
    set_init(key, initializer);
    return key;
  }

  /**
   * @brief      Sets a world initializer.
   *
   * @param[in]  key  The key
   * @param[in]  initializer  The initializer
   */
  void set_init(const std::string &key, const Init &initializer) {
    initializers[key] = initializer;
  }

  /**
   * @brief Remove the last added world initializer.
   *
   * @param[in]  key  The key
   */
  void remove_init(const std::string &key) { initializers.erase(key); }

  /**
   * @brief      Removed all initializers
   */
  void clear_inits() { initializers.clear(); }

  /**
   * @brief      Gets the world initializers.
   *
   * @return     The initializers.
   */
  const Inits &get_inits() const { return initializers; }

  /**
   * @brief      Gets the groups.
   *
   * @return     The groups.
   */
  const Groups &get_groups() const { return groups; }

  /**
   * @brief      Gets a group.
   *
   * @param[in]  index The index
   *
   * @return     The group or null if the index is not defined.
   */
  std::shared_ptr<Group> get_group(size_t index) const {
    if (index < groups.size()) {
      return groups[index];
    }
    return nullptr;
  }

  /**
   * @brief      Adds a group.
   *
   * @param[in]  group  The group
   */
  void add_group(const std::shared_ptr<Group> &group) {
    groups.push_back(group);
  }

  /**
   * @brief Remove the added group.
   *
   * @param[in] group The group
   *
   */
  void remove_group(const std::shared_ptr<Group> &group) {
    groups.erase(std::remove(groups.begin(), groups.end(), group),
                 groups.end());
  }

  /**
   * @brief Remove the added group.
   *
   * @param[in] index The index
   *
   */
  void remove_group_at_index(size_t index) {
    if (index < groups.size()) {
      groups.erase(std::next(groups.begin(), index));
    }
  }

  /**
   * @brief      Remove all groups
   */
  void clear_groups() { groups.clear(); }

  /**
   * Obstacles
   */
  std::vector<Disc> obstacles;
  /**
   * Walls to add
   */
  std::vector<LineSegment> walls;

  /**
   * @brief      Returns the samplers that this
   * scenario uses for its properties.
   *
   * @return     The samplers.
   */
  const PropertySamplers &get_property_samplers() const {
    return property_samplers;
  }

  /**
   * @brief      Adds a property sampler.
   *
   * @param[in]  name  The name of the property
   * @param[in]  value The sampler
   *
   */
  void add_property_sampler(const std::string &name,
                            const std::shared_ptr<PropertySampler> &value) {
    if (value && value->type_name == get_property_type_name(name)) {
      property_samplers[name] = value;
    }
  }

  /**
   * @brief      Removes a property sampler.
   *
   * @param[in]  name The name of the property
   *
   */
  void remove_property_sampler(const std::string &name) {
    property_samplers.erase(name);
  }

  /**
   * @brief      Clears the property samplers
   *
   */
  void clear_property_samplers() { property_samplers.clear(); }

  void reset(std::optional<unsigned> index = std::nullopt) {
    for (auto &[k, v] : property_samplers) {
      if (v)
        v->reset(index, true);
    }
  }

  /**
   * @brief      Sets world attributes from scenario properties.
   *
   * Sets an attribute with the current value for each (registered) property.
   *
   * @param      world  The world
   */
  void set_attributes(World *world);

  /**
   * An optional bounding box
   */
  std::optional<sim::BoundingBox> bounding_box;

private:
  Inits initializers;

  /**
   * Groups
   */
  std::vector<std::shared_ptr<Group>> groups;

  /**
   * A map of property samplers ``name -> sampler``
   * used configure the sampled object.
   */
  PropertySamplers property_samplers;

  std::string key_for_next_init() {
    for (size_t i = 0; i <= initializers.size(); i++) {
      const auto key = std::to_string(i);
      if (!initializers.count(key)) {
        return key;
      }
    }
    return "";
  }
};

} // namespace navground::sim

#endif // NAVGROUND_SIM_SCENARIO_H
