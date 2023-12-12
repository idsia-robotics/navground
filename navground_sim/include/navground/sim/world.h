/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_WORLD_H_
#define NAVGROUND_SIM_WORLD_H_

#include <geos/geom/Envelope.h>
#include <geos/index/strtree/TemplateSTRtree.h>

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "navground/core/states/geometric.h"
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

inline BoundingBox envelop(const Vector2 &position, float radius) {
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
                                                      float radius);

float penetration_inside_line(const LineSegment &line, const Vector2 &center,
                              float radius);

float penetration_inside_disc(const Disc &disc, const Vector2 &center,
                              float radius);

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
  Obstacle(const Vector2 &position, float radius)
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

  /**
   * The disc.
   */
  Disc disc;
};

/**
 * @brief      Ghost agents used in worlds that have a lattice.
 */
struct NAVGROUND_SIM_EXPORT Ghost : Entity, Neighbor {
  explicit Ghost(Agent *agent)
      : Entity(agent->uid),
        Neighbor(agent->pose.position, agent->radius, agent->twist.velocity,
                 agent->id) {}
};

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

  using Lattice = std::optional<std::tuple<float, float>>;
  using Callback = std::function<void()>;

  using A = Agent;

  virtual ~World() = default;

  /**
   * @brief      Constructs a new instance.
   */
  explicit World()
      : agents(),
        obstacles(),
        walls(),
        agent_index(nullptr),
        collisions(),
        entities(),
        ready(false),
        time(0.0f),
        has_lattice(false),
        callbacks() {}

  /**
   * @brief      Updates world for a single time step.
   *
   * @param[in]  time_step  The time step
   */
  void update(float time_step);
  /**
   * @brief      Updates world for a single time step 
   *             without actuation and collisions resolution.
   *
   * @param[in]  time_step  The time step
   * @param[in]  advance_time  Whenever to advance time too.
   */
  void update_dry(float time_step, bool advance_time = true);
  /**
   * @brief      Actuate then controllers and perform collisions resolutions.
   *
   * @param[in]  time_step  The duration of each time step
   */
  void actuate(float time_step);
  /**
   * @brief      Updates the world for one or more time steps
   *
   * @param[in]  steps      The number of steps
   * @param[in]  time_step  The duration of each time step
   */
  void run(unsigned steps, float time_step);
  /**
   * @brief      Updates the world until a condition is satisfied
   *
   * @param[in]  condition  The condition
   * @param[in]  time_step  The duration of each time step
   */
  void run_until(std::function<bool()> condition, float time_step);
  /**
   * @brief      Gets the simulation time.
   *
   * @return     The simulation time.
   */
  float get_time() const { return time; }

  /**
   * @brief      Gets the simulation step.
   *
   * @return     The simulation step.
   */
  unsigned get_step() const { return step; }

  /**
   * @brief      Adds an agent to the world.
   *
   * @param[in]  agent  The agent
   */
  void add_agent(const std::shared_ptr<Agent> &agent);
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
   * @brief      Gets all agents in this world.
   *
   * @return     All agents.
   */
  const std::vector<std::shared_ptr<Agent>> &get_agents() const;

  /**
   * @brief      Gets all neighbor of an agent (ghosts included)
   *
   * @param[in]  agent     The agent
   * @param[in]  distance  The radius of the neighborhood
   *
   * @return     All neighbor within a circle of radius ``radius`` centered around the agent.
   */
  std::vector<Neighbor> get_neighbors(const Agent * agent, float distance) const;

  /**
   * @brief      Gets all agents in a bounding box.
   *
   * @param[in]  bb    The bounding box specified in world-fixed
   * coordinates
   *
   * @return     All agents that lie in a bounding box.
   */
  std::vector<Agent *> get_agents_in_region(const BoundingBox &bb) const;
  /**
   * @brief      Gets all obstacles in this world.
   *
   * @return     All obstacles.
   */
  const std::vector<Obstacle> & get_obstacles() const;
  /**
   * @brief      Gets all disc shaped static obstacles in this world.
   *
   * @return     All obstacles.
   */
  std::vector<Disc> get_discs() const;
  /**
   * @brief      Gets all agents in a bounding box.
   *
   * @param[in]  bb    The bounding box specified in world-fixed
   *
   * @return     All obstacles that lie in a bounding box
   */
  std::vector<Disc> get_static_obstacles_in_region(const BoundingBox &bb) const;
  /**
   * @brief      Gets all walls in this world.
   *
   * @return     All walls.
   */
  const std::vector<Wall> & get_walls() const;
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
  std::vector<LineSegment *> get_line_obstacles_in_region(const BoundingBox &bb) const;
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
   * @brief      Calculates the safety violation,
   * i.e. the maximal penetration of a neighbor or obstacle in the safety margin
   * of the agent.
   *
   * @param[in]  agent  The agent
   *
   * @return     The safety violation or 0 if no violation.
   */
  float compute_safety_violation(const Agent *agent) const;

  /**
   * @brief      Sets the random seed
   *
   * @param[in]  seed  The random seed
   */
  static void set_seed(unsigned seed);

  /**
   * @brief      Check if all agents are idle
   * (i.e., their tasks are done and their controller are idle).
   *
   * @return     True if all agents are idle
   */
  bool agents_are_idle() const;

  /**
   * @brief      Move agents so that they do not overlap anymore with themselves
   * or with any obstacle
   *
   * @param[in]  minimal_distance    The minimal distance
   * @param[in]  with_safety_margin  Whether the safety margin should be added
   * to the minimal distance
   * @param[in]  max_iterations  The maximal number of iterations to perform.
   */
  void space_agents_apart(float minimal_distance = 0.0f,
                          bool with_safety_margin = false,
                          unsigned max_iterations = 10);

  // TODO(Jerome): should be private but it is needed by corridor to ensure
  // that the controller is correctly set.
  void prepare();

  Lattice get_lattice(unsigned index) const;

  void set_lattice(unsigned index, const Lattice &value);

  std::vector<Vector2> lattice_grid() const;

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
  Entity * get_entity(unsigned uid) {
    if (entities.count(uid)) {
      return entities.at(uid);
    }
    return nullptr;
  }

  /**
   * @brief      Adds a callback to be executed after each simulation step.
   *
   * @param[in]  value  The callback
   */
  void add_callback(const Callback& value) { callbacks.push_back(value); }
  void reset_callbacks() { callbacks.clear(); }

 private:
  void record_collision(const Entity *e1, const Entity *e2);

  bool resolve_collision(Agent *a1, Agent *a2, float margin = 0.0f);

  // TODO(J): avoid repetitions
  bool resolve_collision(Agent *agent, Disc *disc, float margin = 0.0f);

  bool resolve_collision(Agent *agent, LineSegment *line, float margin = 0.0f);

  void update_collisions();

  void update_agents_strtree();

  void update_static_strtree();

  bool space_agents_apart_once(float minimal_distance, bool with_safety_margin);

  void update_agent_collisions(Agent *a1);

  void wrap_agents_on_lattice();

  void update_agent_ghosts();

  void add_entity(Entity * entity);

  std::vector<std::shared_ptr<Agent>> agents;
  std::vector<Obstacle> obstacles;
  std::vector<Wall> walls;
  std::vector<Ghost> ghosts;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<Agent *>> agent_index;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<Ghost *>>
      ghost_index;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<Obstacle *>>
      obstacles_index;
  std::shared_ptr<geos::index::strtree::TemplateSTRtree<Wall *>> walls_index;
  std::vector<geos::geom::Envelope> agent_envelops;
  std::vector<geos::geom::Envelope> static_envelops;
  std::vector<geos::geom::Envelope> ghost_envelops;
  std::set<std::tuple<const Entity *, const Entity *>> collisions;
  std::map<unsigned, Entity *> entities;
  bool ready;
  unsigned step;
  float time;
  bool has_lattice;
  std::array<Lattice, 2> lattice;
  std::vector<Callback> callbacks;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_WORLD_H_ */
