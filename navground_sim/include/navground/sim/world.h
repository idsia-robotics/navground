/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_WORLD_H_
#define NAVGROUND_SIM_WORLD_H_

#include <geos/geom/Envelope.h>
#include <geos/index/strtree/TemplateSTRtree.h>

#include <algorithm>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "navground/core/states/geometric.h"
#include "navground/core/types.h"
#include "navground/sim/agent.h"
#include "navground/sim/entity.h"
#include "navground_sim_export.h"

using navground::core::Disc;
using navground::core::LineSegment;
using navground::core::Neighbor;
using navground::core::Vector2;

namespace navground::sim {

/**
 * A rectangulare region
 */
using BoundingBox = geos::geom::Envelope;

inline BoundingBox envelop(const Vector2 &position, ng_float_t radius) {
  return {position[0] - radius, position[0] + radius, position[1] - radius,
          position[1] + radius};
}

// TODO(Jerome): specify direction

/**
 * @brief      Computes the com-penetration of a disc and a line segment
 *
 * @param[in]  line    The line
 * @param[in]  center  The disc center
 * @param[in]  radius  The disc radius
 *
 * @return     The penetration vector, i.e. the shortest shift that would remove
 * overlapping, or  in case  the is not overlap.
 */
std::optional<Vector2> penetration_vector_inside_line(const LineSegment &line,
                                                      const Vector2 &center,
                                                      ng_float_t radius);

ng_float_t penetration_inside_line(const LineSegment &line,
                                   const Vector2 &center, ng_float_t radius);

ng_float_t penetration_inside_disc(const Disc &disc, const Vector2 &center,
                                   ng_float_t radius);

/**
 * @brief      A static wall.
 *
 * Currently, only line segment are valid shapes of walls.
 */
struct NAVGROUND_SIM_EXPORT Wall : Entity {
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  p1    The line segment start vertex
   * @param[in]  p2    The line segment end vertex
   */
  Wall(const Vector2 &p1, const Vector2 &p2) : Entity(), line(p1, p2) {}
  /**
   * @brief      Constructs a new instance.
   */
  Wall() : Entity(), line() {}
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  ls    A line segment
   */
  Wall(const LineSegment &ls) : Entity(), line(ls) {}
  /**
   * @brief      LineSegment conversion operator.
   */
  operator LineSegment() const { return line; }

  /**
   * The line segment
   */
  LineSegment line;
};

/**
 * @brief      A static obstacle with circular shape
 */
struct NAVGROUND_SIM_EXPORT Obstacle : Entity {
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  position  The position of the circle
   * @param[in]  radius    The radius of the circle
   */
  Obstacle(const Vector2 &position, ng_float_t radius)
      : Entity(), disc(position, radius) {}
  /**
   * @brief      Constructs a new instance.
   */
  Obstacle() : Entity(), disc() {}
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  disc  A disc
   */
  Obstacle(const Disc &disc) : Entity(), disc(disc) {}
  /**
   * @brief      Disc conversion operator.
   */
  operator Disc() const { return disc; }

  Disc get_translated_disc(const Vector2 &delta) const {
    return Disc(disc.position + delta, disc.radius);
  }

  /**
   * The disc.
   */
  Disc disc;
};

/**
 * @brief      Ghost agents used in worlds that have a lattice.
 */
// struct NAVGROUND_SIM_EXPORT Ghost : Entity, Neighbor {
//   explicit Ghost(Agent *agent)
//       : Entity(agent->uid),
//         Neighbor(agent->pose.position, agent->radius, agent->twist.velocity,
//                  agent->id) {}
// };

/**
 * @brief      This class describes a world.
 *
 * A world implements the core part of the simulation.
 * It holds a collection of entities as walls, obstacles, and agents.
 *
 * After setting up the world entities, users call \ref update or \ref run
 * to perform one or more simulation steps, where
 *
 * 1. each agent updates its control
 *
 * 2. each agent perform actuate its control command
 *
 * 3. collisions are checked and resolved
 *
 * 4. time is advanced
 *
 * World simulation uses a simple collision model that does not attempt to be
 * realistic but should be computationally efficient.
 *
 * 1. first runs a broad-phase using
 *    `libgeos STR Trees
 * <https://libgeos.org/doxygen/classgeos_1_1index_1_1strtree_1_1STRtree.html>`_
 *    where potential collisions are found using rectangular bounding boxes.
 * 2. then it runs a narrow-phase using the simple exact geometric shape of the
 * obstacles and record pairs of entities that are in collision.
 * 3. finally it resolves collisions by colliding moving minimally entities away
 * from each other, setting to zero the component of their velocities that would
 * attract them together.
 */
class NAVGROUND_SIM_EXPORT World {
 public:
  friend struct Experiment;

  using Lattice = std::optional<std::tuple<ng_float_t, ng_float_t>>;
  using Callback = std::function<void()>;
  using TerminationCondition = std::function<bool(const World *world)>;

  using A = Agent;

  virtual ~World() = default;

  /**
   * @brief      Constructs a new instance.
   */
  explicit World()
      : agents_strtree_is_updated(false),
        static_strtree_is_updated(false),
        agents(),
        obstacles(),
        walls(),
        agent_index(nullptr),
        collisions(),
        entities(),
        ready(false),
        step(0),
        time(0),
        has_lattice(false),
        callbacks(),
        termination_condition(),
        _seed(0),
        _generator(_seed) {}

  /**
   * @brief      Updates world for a single time step.
   *
   * @param[in]  time_step  The time step
   */
  void update(ng_float_t time_step);
  /**
   * @brief      Updates world for a single time step
   *             without actuation and collisions resolution.
   *
   * @param[in]  time_step  The time step
   * @param[in]  advance_time  Whenever to advance time too.
   */
  void update_dry(ng_float_t time_step, bool advance_time = true);
  /**
   * @brief      Actuate then controllers and perform collisions resolutions.
   *
   * @param[in]  time_step  The duration of each time step
   */
  void actuate(ng_float_t time_step);
  /**
   * @brief      Updates the world for one or more time steps
   *
   * @param[in]  steps      The number of steps
   * @param[in]  time_step  The duration of each time step
   */
  void run(unsigned steps, ng_float_t time_step);
  /**
   * @brief      Updates the world until a condition is satisfied
   *
   * @param[in]  condition  The condition
   * @param[in]  time_step  The duration of each time step
   */
  void run_until(std::function<bool()> condition, ng_float_t time_step);
  /**
   * @brief      Gets the simulation time.
   *
   * @return     The simulation time.
   */
  ng_float_t get_time() const { return time; }

  /**
   * @brief      Gets the simulation step.
   *
   * @return     The simulation step.
   */
  unsigned get_step() const { return step; }

  /**
   * @brief      Sets the simulation time.
   *
   * @private
   *
   * For play-back
   *
   * @param[in]     The simulation time.
   */
  void set_time(ng_float_t value) { time = std::max<ng_float_t>(0, value); }

  /**
   * @brief      Sets the simulation step.
   *
   * @private
   *
   * For play-back
   *
   * @param[in]    The simulation step.
   */
  void set_step(unsigned value) { step = value; }

  /**
   * @brief      Adds an agent to the world.
   *
   * @param[in]  agent  The agent
   */
  void add_agent(const std::shared_ptr<Agent> &agent);
  /**
   * @brief      Remove an agent from the world.
   *
   * @param[in]  agent  The agent
   */
  void remove_agent(Agent *agent);
  /**
   * @brief      Remove an agent from the world.
   *
   * @param[in]  uid  The uid of the agent
   */
  void remove_agent_with_uid(unsigned uid);
  /**
   * @brief      Adds a line to the world as a wall
   *
   * @param[in]  line  The line
   */
  void add_wall(const LineSegment &line);
  /**
   * @brief      Adds a wall to the world
   *
   * @param[in]  wall  The wall
   */
  void add_wall(const Wall &wall);
  /**
   * @brief      Adds a disc the world as a static obstacle
   *
   * @param[in]  disc  The disc
   */
  void add_obstacle(const Disc &disc);
  /**
   * @brief      Adds a static obstacle the world
   *
   * @param[in]  obstacle  The obstacle
   */
  void add_obstacle(const Obstacle &obstacle);

  /**
   * @brief      Adds a random obstacles in the world bounding box
   *
   * It iteratively try to sample a disc that is far enough of any other item.
   * If the sampling fails, it counts as a try and the process stop after either
   * enough obstacles have been added or enough tries performed.
   *
   * @param[in]  number      The number of obstacles
   * @param[in]  min_radius  The minimum radius of obstacles
   * @param[in]  max_radius  The maximum radius of obstacles
   * @param[in]  margin      The minimal distance to other obstacles or agents.
   *                         For agents, it's additional to their safety margin.
   * @param[in]  max_tries   The maximum tries before terminating.
   */
  void add_random_obstacles(unsigned number, ng_float_t min_radius,
                            ng_float_t max_radius, ng_float_t margin = 0.0,
                            unsigned max_tries = 1000);

  /**
   * @brief      Gets all agents in this world.
   *
   * @return     All agents.
   */
  const std::vector<std::shared_ptr<Agent>> &get_agents() const;

  /**
   * @brief      Gets all neighbor of an agent
   *
   * @param[in]  agent          The agent
   * @param[in]  distance       The radius of the neighborhood
   * @param[in]  ignore_lattice Whether to ignore the lattice
   *                            when computing neighbors
   *
   * @return     All neighbor within a circle of radius ``radius`` centered
   * around the agent.
   */
  std::vector<Neighbor> get_neighbors(const Agent *agent, ng_float_t distance,
                                      bool ignore_lattice = false);

  /**
   * @brief      Split a bounding box by the lattice-induced grid
   *
   * @param[in]  bounding_box    The bounding box
   * @param[in]  ignore_lattice  Indicates if the lattice is to be ignored
   *
   * @return     A vector of pairs of translation (along the lattice)
   *             and subset of the bounding box that fit in that lattice cell.
   */
  std::vector<std::tuple<Vector2, BoundingBox>> subdivide_bounding_box(
      const BoundingBox &bounding_box, bool ignore_lattice = false) const;

  /**
   * @brief      Gets all agents in a bounding box.
   *
   * @param[in]  bb    The bounding box specified in world-fixed
   * coordinates
   *
   * @return     All agents that lie in a bounding box.
   */
  std::vector<Agent *> get_agents_in_region(const BoundingBox &bb);

  /**
   * @brief      Gets all obstacles in a bounding box.
   *
   * @param[in]  bb    The bounding box specified in world-fixed
   * coordinates
   *
   * @return     All obstacles that lie in a bounding box.
   */
  std::vector<Obstacle *> get_obstacles_in_region(const BoundingBox &bb);

  /**
   * @brief      Gets all obstacles in this world.
   *
   * @return     All obstacles.
   */
  const std::vector<std::shared_ptr<Obstacle>> &get_obstacles() const;
  /**
   * @brief      Gets all disc shaped static obstacles in this world.
   *
   * @param[in]  ignore_lattice Whether to ignore the lattice
   *
   * @return     All obstacles.
   */
  std::vector<Disc> get_discs(bool ignore_lattice = false) const;
  /**
   * @brief      Gets all agents in a bounding box.
   *
   * @param[in]  bb    The bounding box specified in world-fixed
   * @param[in]  ignore_lattice Whether to ignore the lattice
   *                            when computing neighbors
   *
   * @return     All obstacles that lie in a bounding box
   */
  std::vector<Disc> get_discs_in_region(const BoundingBox &bb,
                                        bool ignore_lattice = false);
  /**
   * @brief      Gets all walls in this world.
   *
   * @return     All walls.
   */
  const std::vector<std::shared_ptr<Wall>> &get_walls() const;
  /**
   * @brief      Gets all line obstacles in this world.
   *
   * @return     All ine obstacles.
   */
  std::vector<LineSegment> get_line_obstacles() const;
  /**
   * @brief      Gets all walls in a bounding box.
   *
   * @param[in]  bb    The bounding box specified in world-fixed
   *
   * @return     All walls that lie in a bounding box
   */
  std::vector<LineSegment *> get_line_obstacles_in_region(
      const BoundingBox &bb) const;
  /**
   * @brief      Replaces all obstacles.
   *
   * @param[in]  obstacles  The new obstacles
   */
  void set_obstacles(const std::vector<Disc> &obstacles);
  /**
   * @brief      Replaces all walls.
   *
   * @param[in]  walls  The new walls
   */
  void set_walls(const std::vector<LineSegment> &walls);
  /**
   * @brief      Gets the colliding pairs computed during the last simulation
   * step.
   *
   * @return     The colliding pair of entities.
   */
  const std::set<std::tuple<const Entity *, const Entity *>> &get_collisions()
      const {
    return collisions;
  }

  /**
   * @brief      Sets the colliding pairs computed during the last simulation
   * step.
   *
   * Use this method to update collisions computed externally or recorded.
   *
   * @private
   *
   * @param[in]     The colliding pair of entities.
   */
  void set_collisions(const std::set<std::tuple<Entity *, Entity *>> &value) {
    collisions.clear();
    for (const auto &[e1, e2] : value) {
      record_collision(e1, e2);
    }
  }

  /**
   * @brief      Calculates the safety violation,
   * i.e. the maximal penetration of a neighbor or obstacle in the safety margin
   * of the agent.
   *
   * @param[in]  agent  The agent
   * @param[in]  agent  The safety margin.
   *                    If not set, it will default to the agent's behavior
   * safety margin.
   *
   * @return     The safety violation or 0 if no violation.
   */
  ng_float_t compute_safety_violation(
      const Agent *agent, std::optional<float> safety_margin = std::nullopt);

  /**
   * @brief      The random generator shared by all distribution
   * used to generate and simulate this world
   *
   * @return     The random generator
   */
  RandomGenerator &get_random_generator();

  /**
   * @brief      Sets the random generator shared by all distribution
   * used to generate and simulate this world
   *
   * @param[in]  value  The desired random generator
   */
  void set_random_generator(RandomGenerator &value);

  /**
   * @brief      Copy the random generator from another world
   *
   * @param[in]  world  The world
   */
  void copy_random_generator(World &world) {
    set_random_generator(world.get_random_generator());
  }

  /**
   * @brief      Gets the random seed.
   *
   * @return     The random seed.
   */
  unsigned get_seed() const;

  /**
   * @brief      Sets the random seed
   *
   * @param[in]  seed  The random seed
   */
  void set_seed(unsigned seed);

  /**
   * @brief      Check if all agents are idle
   * (i.e., their tasks are done and their controller are idle).
   *
   * @return     True if all agents are idle
   */
  bool agents_are_idle() const;

  /**
   * @brief      Check if all agents are idle or stuck
   * (i.e., they are no moving because they task is done or they are deadlocked)
   *
   * @return     True if all agents are idle or stuck
   */
  bool agents_are_idle_or_stuck() const;

  /**
   * @brief      Move agents so that they do not overlap anymore with themselves
   * or with any obstacle
   *
   * @param[in]  minimal_distance    The minimal distance
   * @param[in]  with_safety_margin  Whether the safety margin should be added
   * to the minimal distance
   * @param[in]  max_iterations  The maximal number of iterations to perform.
   */
  void space_agents_apart(ng_float_t minimal_distance = 0,
                          bool with_safety_margin = false,
                          unsigned max_iterations = 10);

  // TODO(Jerome): should be private but it is needed by corridor to ensure
  // that the controller is correctly set.
  void prepare();

  /**
   * @brief      Gets the periodic lattice.
   *
   * @param[in]  axis  The axis (0 for x, 1 for y)
   *
   * @return     An optional tuple of points that define a
   *             periodic lattice that wraps the selected axis.
   */
  Lattice get_lattice(unsigned axis) const;

  /**
   * @brief      Sets the periodic lattice.
   *
   * @param[in]  axis  The axis (0 for x, 1 for y)
   * @param[in]  value  An optional tuple of points that define a
   *             periodic lattice that wraps the selected axis.
   *             Pass none to unset the lattice and remove wrapping.
   */
  void set_lattice(unsigned axis, const Lattice &value);

  /**
   * @brief      The N=0, 1, or 2 vectors that define the
   *             lattice, e.g., ``{delta_x, -delta_x}``
   *             if only the axis=0 lattice is set.
   *
   * @param[in]  Whether to include the zero vector
   * @param[in]  Whether to use 8-connectivity instead of 4-connectivity
   *
   * @return     A vector of 2D vectors
   */
  std::vector<Vector2> get_lattice_grid(bool include_zero = true,
                                        bool c8 = true) const;

  /**
   * @brief      Gets the lattice cell.
   *
   * @return     The lattice cell.
   */
  BoundingBox get_lattice_bounding_box() const;

  /**
   * @brief      Check if two entities are currently in collision
   *
   * @param[in]  e1    The first entity
   * @param[in]  e2    The second entity
   *
   * @return     True if they are in collision.
   */
  bool in_collision(const Entity *e1, const Entity *e2) const;

  /**
   * @brief      Find an entity by identifier
   *
   * @param[in]  uid   The entity uid
   *
   * @return     The entity or nullptr if not found.
   */
  Entity *get_entity(unsigned uid) {
    if (entities.count(uid)) {
      return entities.at(uid);
    }
    return nullptr;
  }

  /**
   * @brief      Find an agent by identifier
   *
   * @param[in]  uid   The agent uid
   *
   * @return     The agent or nullptr if not found.
   */
  Agent *get_agent(unsigned uid) {
    return dynamic_cast<Agent *>(get_entity(uid));
  }

  /**
   * @brief      Adds a callback to be executed after each simulation step.
   *
   * @param[in]  value  The callback
   */
  void add_callback(const Callback &value) { callbacks.push_back(value); }
  /**
   * @brief      Clear all the callbacks
   */
  void reset_callbacks() { callbacks.clear(); }

  /**
   * @brief      Sets a condition to terminate simulations
   *
   * @param[in]  value  The desired condition.
   */
  void set_termination_condition(
      const std::optional<TerminationCondition> &value) {
    termination_condition = value;
  }

  /**
   * @brief      Returns whether there is a termination condition set.
   *
   * @return     True if a termination condition has been set.
   */
  bool has_termination_condition() const { return bool(termination_condition); }

  /**
   * @brief      Checks whether the simulation should terminate
   *
   * @return     True if a termination condition is set and evaluates to true.
   */
  bool should_terminate() const {
    if (termination_condition) {
      return (*termination_condition)(this);
    }
    return false;
  }

  /**
   * @brief      Gets the agents that had a collision after ``now - duration``.
   *
   * @param[in]  duration  The duration
   *
   * @return     The agents in collision.
   */
  std::vector<Agent *> get_agents_in_collision(ng_float_t duration = 0.0) const;
  /**
   * @brief      Gets the agents that are in stuck since ``now - duration``.
   *
   * @param[in]  duration  The duration
   *
   * @return     The agents in deadlock.
   */
  std::vector<Agent *> get_agents_in_deadlock(ng_float_t duration = 0.0) const;

  /**
   * @brief      Snap agents' twists smaller than epsilon to zero.
   *
   * @param[in]  epsilon  The tolerance
   */
  void snap_twists_to_zero(ng_float_t epsilon = 1e-6) const;

  /**
   * @brief      Searches for the index of an agent.
   *
   * @param[in]  agent  The agent
   *
   * @return     The index of this agent in the world agents list
   *             or null if not found.
   */
  std::optional<unsigned> index_of_agent(const Agent *agent) const {
    auto it = std::find_if(agents.begin(), agents.end(),
                           [&agent](const std::shared_ptr<Agent> &other) {
                             return other.get() == agent;
                           });
    if (it != std::end(agents)) {
      return std::distance(agents.begin(), it);
    }
    return std::nullopt;
  }

  /**
   * @brief      Gets the bounding box that contains all agents, obstacles and
   * walls.
   *
   * @return     The computed bounding box.
   */
  BoundingBox get_minimal_bounding_box() const;

  /**
   * @brief      Sets the bounding box.
   *
   * @param[in]  value  The desired value. Pass none to unset the bounding box.
   */
  void set_bounding_box(const std::optional<BoundingBox> &value) {
    bb = value;
  };

  /**
   * @brief      Gets the bounding box.
   *
   * @return     The bounding box that has been set with \ref set_bounding_box
   *             or \ref get_minimal_bounding_box in case none is set.
   */
  BoundingBox get_bounding_box() const {
    if (bb) {
      return *bb;
    }
    return get_minimal_bounding_box();
  }

 private:
  void record_collision(Entity *e1, Entity *e2);

  bool resolve_collision(Agent *a1, Agent *a2, Vector2 delta = Vector2::Zero(),
                         ng_float_t margin = 0);

  // TODO(J): avoid repetitions
  bool resolve_collision(Agent *agent, Disc *disc,
                         Vector2 delta = Vector2::Zero(),
                         ng_float_t margin = 0);

  bool resolve_collision(Agent *agent, LineSegment *line,
                         ng_float_t margin = 0);

  void update_collisions();

  void update_agents_strtree();

  void update_static_strtree();

  bool space_agents_apart_once(ng_float_t minimal_distance,
                               bool with_safety_margin);

  void update_agent_collisions(Agent *a1);

  void wrap_agents_on_lattice();

  // void update_agent_ghosts();

  void add_entity(Entity *entity);

  void remove_entity(Entity *entity);

  void check_static_strtree();
  void check_agents_strtree();
  bool agents_strtree_is_updated;
  bool static_strtree_is_updated;

  std::vector<std::shared_ptr<Agent>> agents;
  std::vector<std::shared_ptr<Obstacle>> obstacles;
  std::vector<std::shared_ptr<Wall>> walls;
  // std::vector<Ghost> ghosts;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<Agent *>> agent_index;
  // std::shared_ptr<geos::index::strtree::TemplateSTRtree<Ghost *>>
  // ghost_index;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<Obstacle *>>
      obstacles_index;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<Wall *>> walls_index;
  std::vector<geos::geom::Envelope> agent_envelops;
  std::vector<geos::geom::Envelope> static_envelops;
  // std::vector<geos::geom::Envelope> ghost_envelops;
  std::set<std::tuple<const Entity *, const Entity *>> collisions;
  std::map<unsigned, Entity *> entities;
  bool ready;
  unsigned step;
  ng_float_t time;
  bool has_lattice;
  std::array<Lattice, 2> lattice;
  std::vector<Callback> callbacks;
  std::optional<TerminationCondition> termination_condition;
  unsigned _seed;
  RandomGenerator _generator;
  std::optional<BoundingBox> bb;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_WORLD_H_ */
