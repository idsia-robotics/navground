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
 * @brief      A scenario describes a distribution of \ref World
 * that can be sampled to perform an experiment.
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
   * @brief      Constructs a new instance.
   *
   * @param[in]  inits  The collection of world initializers to use.
   */
  explicit Scenario(const Inits &inits = {})
      : groups(), obstacles(), walls(), property_samplers(),
        initializers(inits) {}

  /**
   * @brief      Initializes the world.
   *
   * @param      world The world
   * @param      seed  The random seed
   */
  virtual void init_world(World *world, std::optional<int> seed = std::nullopt);

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
   * @brief      Adds a group.
   *
   * @param[in]  group  The group
   */
  void add_group(const std::shared_ptr<Group> &group) {
    groups.push_back(group);
  }

  /**
   * @brief Remove the last added group.
   *
   */
  void remove_group() {
    if (groups.size()) {
      groups.pop_back();
    }
  }

  /**
   * @brief      Remove all groups
   */
  void clear_groups() { groups.clear(); }

  /**
   * Groups
   */
  std::vector<std::shared_ptr<Group>> groups;
  /**
   * Obstacles
   */
  std::vector<Disc> obstacles;
  /**
   * Walls to add
   */
  std::vector<LineSegment> walls;

  /**
   * A map of property samplers ``name -> sampler``
   * used configure the sampled object.
   */
  std::map<std::string, std::shared_ptr<PropertySampler>> property_samplers;

  void reset(std::optional<unsigned> index = std::nullopt) {
    for (auto &[k, v] : property_samplers) {
      if (v)
        v->reset(index);
    }
  }

  /**
   * An optional bounding box
   */
  std::optional<sim::BoundingBox> bounding_box;

private:
  Inits initializers;

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
