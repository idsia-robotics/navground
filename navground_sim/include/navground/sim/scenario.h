/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIO_H
#define NAVGROUND_SIM_SCENARIO_H

#include <functional>
#include <optional>
#include <vector>

#include "navground/core/property.h"
#include "navground/core/register.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/world.h"
#include "navground/sim/export.h"

using navground::core::Disc;
using navground::core::HasProperties;
using navground::core::HasRegister;
using navground::core::LineSegment;

namespace navground::sim {

/**
 * @brief      A scenario describes a distribution of \ref World
 * that can be sampled to perform an experiment.
 */
struct NAVGROUND_SIM_EXPORT Scenario : virtual public HasProperties,
                                       virtual public HasRegister<Scenario> {
  /**
   * @brief      A group of agents that can be generated and added to the world.
   */
  struct Group {
    /**
     * @brief      Generate and add the agents to the world.
     *
     * @param      world  The world
     */
    virtual void add_to_world(World* world) = 0;
    /**
     * @brief      Resets the agent generator.
     */
    virtual void reset(std::optional<unsigned> index = std::nullopt) = 0;
    virtual ~Group() = default;
  };

  /**
   * A world initializer
   */
  using Init = std::function<void(World*)>;
  /**
   * A collection of world initializers
   */
  using Inits = std::vector<Init>;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  inits  The collection of world initializers to use.
   */
  explicit Scenario(const Inits& inits = {})
      : groups(),
        obstacles(),
        walls(),
        property_samplers(),
        initializers(inits) {}

  // !!!! This may break the auto-registration!
  // std::string get_type() const override { return name; }

  /**
   * @brief      Initializes the world.
   *
   * @param      world The world
   * @param      seed  The random seed
   */
  virtual void init_world(World* world, std::optional<int> seed = std::nullopt);

  /**
   * @brief      Adds a world initializer.
   *
   * @param[in]  f     The initializer
   */
  void add_init(const std::function<void(World*)>& initializer) {
    initializers.push_back(initializer);
  }

  /**
   * @brief      Gets the world initializers.
   *
   * @return     The initializers.
   */
  const Inits& get_initializers() const { return initializers; }

  /**
   * Groups
   */
  std::vector<std::shared_ptr<Group>> groups;
  /**
   * Obstacles
   */
  std::vector<Disc> obstacles;
  /**
   * Walls
   */
  std::vector<LineSegment> walls;

  /**
   * A map of property samplers ``name -> sampler``
   * used configure the sampled object.
   */
  std::map<std::string, std::shared_ptr<PropertySampler>> property_samplers;

  void reset(std::optional<unsigned> index = std::nullopt) {
    for (auto& [k, v] : property_samplers) {
      if (v) v->reset(index);
    }
  }

 private:
  Inits initializers;
};

}  // namespace navground::sim

#endif  // NAVGROUND_SIM_SCENARIO_H
