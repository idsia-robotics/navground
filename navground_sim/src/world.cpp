/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/world.h"

#include "navground/sim/sampling/geometry.h"

using navground::core::Frame;

namespace navground::sim {

void World::set_seed(unsigned value) {
  if (_seed != value) {
    _seed = value;
    _generator.seed(_seed);
  }
}

unsigned World::get_seed() const { return _seed; }

RandomGenerator &World::get_random_generator() { return _generator; }

void World::set_random_generator(RandomGenerator &value) { _generator = value; }

static bool wrap_lattice(ng_float_t &x, ng_float_t from, ng_float_t length) {
  const ng_float_t delta = x - from;
  if (delta > length) {
    x = from + std::fmod(delta, length);
    return true;
  }
  if (delta < 0) {
    x = from + std::fmod(delta, length) + length;
    return true;
  }
  return false;
}

std::optional<Vector2> penetration_vector_inside_line(const LineSegment &line,
                                                      const Vector2 &center,
                                                      ng_float_t radius) {
  ng_float_t y = (center - line.p1).dot(line.e2);
  if (abs(y) < radius) {
    ng_float_t x = (center - line.p1).dot(line.e1);
    if (x < radius + 1e-3 || x > line.length - radius - 1e-3)
      return std::nullopt;
    ng_float_t p = radius - abs(y);
    if (y < 0) p *= -1;
    return p * line.e2;
  }
  return std::nullopt;
}

ng_float_t penetration_inside_line(const LineSegment &line,
                                   const Vector2 &center, ng_float_t radius) {
  ng_float_t y = (center - line.p1).dot(line.e2);
  if (abs(y) < radius) {
    ng_float_t x = (center - line.p1).dot(line.e1);
    if (x < radius + 1e-3 || x > line.length - radius - 1e-3) return 0.0;
    return radius - abs(y);
  }
  return 0;
}

ng_float_t penetration_inside_disc(const Disc &disc, const Vector2 &center,
                                   ng_float_t radius) {
  return std::max<ng_float_t>(
      0, radius + disc.radius - (disc.position - center).norm());
}

void World::update(ng_float_t time_step) {
  if (!ready) {
    prepare();
  }
  for (auto &a : agents) {
    a->update(time_step, time, this);
  }
  for (auto &a : agents) {
    a->actuate(time_step);
  }
  update_agents_strtree();
  update_collisions();
  if (_has_lattice) {
    wrap_agents_on_lattice();
  }
  // bool r = false;

  // if (r) {
  //   update_agents_strtree();
  // }
  time += time_step;
  step++;
  for (const auto &cb : callbacks) {
    cb();
  }
}

void World::snap_twists_to_zero(ng_float_t epsilon) const {
  for (const auto &a : agents) {
    a->twist.snap_to_zero(epsilon);
  }
}

void World::actuate(ng_float_t time_step) {
  if (!ready) {
    prepare();
  }
  for (auto &a : agents) {
    a->actuate(time_step);
  }
  update_agents_strtree();
  update_collisions();
  if (_has_lattice) {
    wrap_agents_on_lattice();
  }
  time += time_step;
  step++;
}

void World::update_dry(ng_float_t time_step, bool advance_time) {
  if (!ready) {
    prepare();
  }
  update_agents_strtree();
  for (auto &a : agents) {
    a->update(time_step, time, this);
  }
  if (advance_time) {
    time += time_step;
    step++;
  }
  // for (const auto &cb : callbacks) {
  //   cb();
  // }
}

void World::add_entity(Entity *entity) { entities[entity->uid] = entity; }

void World::remove_entity(Entity *entity) {
  if (entity) {
    delete entities[entity->uid];
  }
}

void World::add_agent(const std::shared_ptr<Agent> &agent) {
  if (agent) {
    if (entities.count(agent->uid) == 0) {
      agents.push_back(agent);
      ready = false;
      agents_strtree_is_updated = false;
      add_entity(agent.get());
    } else {
      std::cerr << "This agent was already added!" << std::endl;
    }
  }
}

// TODO(Jerome): will create problems with experiments,
// as they assume that the number of agents is fixed.
void World::remove_agent(Agent *agent) {
  if (agent) {
    remove_entity(agent);
    auto it = std::find_if(agents.begin(), agents.end(),
                           [agent](const auto &p) { return p.get() == agent; });
    if (it != std::end(agents)) {
      agents.erase(it);
    }
  }
}

void World::remove_agent_with_uid(unsigned uid) {
  remove_agent(get_agent(uid));
}

void World::add_wall(const LineSegment &wall) {
  walls.push_back(std::make_shared<Wall>(wall));
  add_entity(walls.back().get());
  ready = false;
  static_strtree_is_updated = false;
}

void World::add_wall(const Wall &wall) {
  if (entities.count(wall.uid) == 0) {
    walls.push_back(std::make_shared<Wall>(wall));
    add_entity(walls.back().get());
    ready = false;
    static_strtree_is_updated = false;
  } else {
    std::cerr << "This wall was already added!" << std::endl;
  }
}

void World::add_obstacle(const Disc &obstacle) {
  obstacles.push_back(std::make_shared<Obstacle>(obstacle));
  add_entity(obstacles.back().get());
  ready = false;
  static_strtree_is_updated = false;
}

void World::add_obstacle(const Obstacle &obstacle) {
  if (entities.count(obstacle.uid) == 0) {
    obstacles.push_back(std::make_shared<Obstacle>(obstacle));
    add_entity(obstacles.back().get());
    ready = false;
    static_strtree_is_updated = false;
  } else {
    std::cerr << "This obstacle was already added!" << std::endl;
  }
}

void World::set_obstacles(const std::vector<Disc> &values) {
  obstacles.clear();
  for (const auto &value : values) {
    add_obstacle(value);
  }
  static_strtree_is_updated = false;
}

// TODO(jerome) (clear)
void World::set_walls(const std::vector<LineSegment> &values) {
  walls.clear();
  for (const auto &value : values) {
    add_wall(value);
  }
  static_strtree_is_updated = false;
}

void World::prepare() {
  // TODO(Jerome) Should only execute it once if not already prepared.
  for (auto &a : agents) {
    a->prepare(this);
  }
  ready = true;
}

void World::run(unsigned steps, ng_float_t time_step) {
  for (size_t i = 0; i < steps; i++) {
    if (should_terminate()) return;
    update(time_step);
  }
}

void World::run_until(std::function<bool()> condition, ng_float_t time_step) {
  while (!condition() && !should_terminate()) {
    update(time_step);
  }
}

const std::vector<std::shared_ptr<Agent>> &World::get_agents() const {
  return agents;
}

std::vector<Agent *> World::get_agents_in_region(const BoundingBox &bb) {
  check_agents_strtree();
  std::vector<Agent *> rs;
  // std::transform(agents.cbegin(), agents.cend(), std::back_inserter(rs),
  // [](const auto & a) { return &a; });
  agent_index->query(bb, rs);
  return rs;
}

std::vector<Obstacle *> World::get_obstacles_in_region(const BoundingBox &bb) {
  check_static_strtree();
  std::vector<Obstacle *> rs;
  obstacles_index->query(bb, rs);
  return rs;
}

// TODO(Jerome): add agent/obstacle radius to narrow phase
std::vector<Neighbor> World::get_neighbors(const Agent *agent,
                                           ng_float_t distance,
                                           bool ignore_lattice) {
  check_agents_strtree();
  std::vector<Neighbor> rs;
  const auto bb = envelop(agent->pose.position, distance);
  const auto bbs = subdivide_bounding_box(bb, ignore_lattice);
  const auto p = agent->pose.position;
  // const auto uid = agent->uid;

  // const unsigned n = agents.size() + (has_lattice ? ghosts.size() : 0);
  // rs.reserve(n);

  for (const auto &[d, lbb] : bbs) {
    const Vector2 &delta = d;
    agent_index->query(lbb, [&rs, &agent, &p, &distance, &delta](Agent *a) {
      if (a != agent &&
          (a->pose.position + delta - p).norm() < distance + a->radius) {
        rs.push_back(a->as_translated_neighbor(delta));
      }
    });
  }
  return rs;
}

// TODO(Jerome) Should cache .. this needs to be fast
// DONE(Jerome): added support for lattice
std::vector<Disc> World::get_discs(bool ignore_lattice) const {
  std::vector<core::Vector2> grid;
  if (!ignore_lattice) {
    grid = get_lattice_grid();
  } else {
    grid = {Vector2()};
  }
  std::vector<Disc> discs(obstacles.size() * grid.size());
  auto it = discs.begin();
  for (const auto &delta : grid) {
    std::transform(
        obstacles.cbegin(), obstacles.cend(), it,
        [&delta](const auto &o) { return o->get_translated_disc(delta); });
    it += obstacles.size();
  }
  return discs;
}

// TODO(Jerome) Should cache .. this needs to be fast
const std::vector<std::shared_ptr<Obstacle>> &World::get_obstacles() const {
  return obstacles;
}

BoundingBox World::get_lattice_bounding_box() const {
  ng_float_t x1, x2, y1, y2;
  if (lattice[0]) {
    x1 = std::get<0>(*lattice[0]);
    x2 = x1 + std::get<1>(*lattice[0]);
  } else {
    x1 = std::numeric_limits<ng_float_t>::lowest();
    x2 = std::numeric_limits<ng_float_t>::max();
  }
  if (lattice[1]) {
    y1 = std::get<0>(*lattice[1]);
    y2 = y1 + std::get<1>(*lattice[1]);
  } else {
    y1 = std::numeric_limits<ng_float_t>::lowest();
    y2 = std::numeric_limits<ng_float_t>::max();
  }
  return BoundingBox(x1, x2, y1, y2);
}

std::vector<std::tuple<Vector2, BoundingBox>> World::subdivide_bounding_box(
    const BoundingBox &bounding_box, bool ignore_lattice) const {
  if (!_has_lattice || ignore_lattice) {
    return {{Vector2(), bounding_box}};
  }
  std::vector<std::tuple<Vector2, BoundingBox>> rs;
  std::vector<core::Vector2> grid = get_lattice_grid();
  BoundingBox lbb = get_lattice_bounding_box();
  for (const auto &delta : grid) {
    BoundingBox bb = bounding_box;
    bb.translate(delta[0], delta[1]);
    BoundingBox i;
    if (lbb.intersection(bb, i)) {
      rs.push_back({-delta, i});
    }
  }
  return rs;
}

// DONE(Jerome): added support for lattice
std::vector<Disc> World::get_discs_in_region(const BoundingBox &bb,
                                             bool ignore_lattice) {
  check_static_strtree();
  std::vector<Disc> discs;
  for (const auto &[d, lbb] : subdivide_bounding_box(bb, ignore_lattice)) {
    const Vector2 &delta = d;
    obstacles_index->query(lbb, [&discs, &delta](Obstacle *o) {
      discs.push_back(o->get_translated_disc(delta));
    });
  }
  return discs;
}

// TODO(Jerome) Should cache .. this needs to be fast
const std::vector<std::shared_ptr<Wall>> &World::get_walls() const {
  return walls;
}

// TODO(Jerome) Should cache .. this needs to be fast
std::vector<LineSegment> World::get_line_obstacles() const {
  std::vector<LineSegment> lines(walls.size());
  std::transform(walls.cbegin(), walls.cend(), lines.begin(),
                 [](const auto &w) { return w->line; });
  return lines;
}

// TODO(J): complete
std::vector<LineSegment *> World::get_line_obstacles_in_region(
    [[maybe_unused]] const BoundingBox &bb) const {
  return {};
}

bool World::resolve_collision(Agent *a1, Agent *a2, Vector2 d2,
                              ng_float_t margin) {
  auto delta = a1->pose.position - a2->pose.position - d2;
  ng_float_t p = delta.norm() - a1->radius - a2->radius - margin;
  if (p > 0) {
    return false;
  }
  auto u = delta / delta.norm();
  auto correction = (-p * 0.5 + 1e-3) * u;
  a1->collision_correction += correction;
  a2->collision_correction -= correction;
  ng_float_t d = a1->twist.velocity.dot(-u);
  if (d > 0) {
    a1->twist.velocity += d * u;
  }
  d = a2->twist.velocity.dot(u);
  if (d > 0) {
    a2->twist.velocity -= d * u;
  }
  return true;
  // auto force = -p * k * delta / delta.norm();
  // std::cout << force << std::endl;
  // a1->collision_force += force;
  // a2->collision_force -= force;
}

// TODO(J): avoid repetitions
bool World::resolve_collision(Agent *agent, Disc *disc, Vector2 d2,
                              ng_float_t margin) {
  auto delta = agent->pose.position - disc->position - d2;
  ng_float_t p = delta.norm() - agent->radius - disc->radius - margin;
  if (p > 0) {
    return false;
  }
  auto u = delta / delta.norm();
  auto correction = (-p * 1.0 + 1e-3) * u;
  agent->collision_correction += correction;
  ng_float_t d = agent->twist.velocity.dot(-u);
  if (d > 0) {
    agent->twist.velocity += d * u;
  }
  return true;
}

bool World::resolve_collision(Agent *agent, LineSegment *line,
                              ng_float_t margin) {
  if (auto p = penetration_vector_inside_line(*line, agent->pose.position,
                                              agent->radius + margin)) {
    auto n = p->norm();
    auto u = *p / n;
    agent->collision_correction += (n + 1e-3) * u;
    ng_float_t d = -agent->twist.velocity.dot(u);
    if (d > 0) {
      agent->twist.velocity += d * u;
    }
    return true;
  }
  return false;
}

void World::record_collision(Entity *e1, Entity *e2) {
  collisions.emplace(e1, e2);
  e1->set_as_colliding_at(time);
  e2->set_as_colliding_at(time);
}

bool World::in_collision(const Entity *e1, const Entity *e2) const {
  return collisions.count({e1, e2}) > 0 || collisions.count({e2, e1}) > 0;
}

std::vector<Agent *> World::get_agents_in_collision(ng_float_t duration) const {
  std::vector<Agent *> rs;
  for (const auto &agent : agents) {
    if (agent->has_been_in_collision_since(time - duration)) {
      rs.push_back(agent.get());
    }
  }
  return rs;
}

std::vector<Agent *> World::get_agents_in_deadlock(ng_float_t duration) const {
  std::vector<Agent *> rs;
  for (const auto &agent : agents) {
    if (agent->has_been_stuck_since(time - duration)) {
      rs.push_back(agent.get());
    }
  }
  return rs;
}

// TODO(J): spare recomputing the envelops?
void World::update_agent_collisions(Agent *a1) {
  // const BoundingBox &bb = agent_envelops[i];
  const BoundingBox bb = envelop(a1->pose.position, a1->radius);
  for (const auto &[d, lbb] : subdivide_bounding_box(bb)) {
    const Vector2 &delta = d;
    agent_index->query(lbb, [this, a1, &delta](Agent *a2) {
      if (a1->uid < a2->uid) {
        if (resolve_collision(a1, a2, delta)) {
          record_collision(a1, a2);
        }
      }
    });
    obstacles_index->query(lbb, [this, a1, &delta](Obstacle *o) {
      if (resolve_collision(a1, &(o->disc), delta)) {
        record_collision(a1, o);
      }
    });
  }
  walls_index->query(bb, [this, a1](Wall *w) {
    if (resolve_collision(a1, &(w->line))) {
      record_collision(a1, w);
    }
  });
}

void World::update_collisions() {
  check_agents_strtree();
  check_static_strtree();
  collisions.clear();
  for (const auto &agent : agents) {
    update_agent_collisions(agent.get());
  }
  for (const auto &agent : agents) {
    agent->pose.position += agent->collision_correction;
    agent->collision_correction = Vector2::Zero();
  }
  // if (!collisions.empty()) {
  //   update_agents_strtree();
  // }
  // TODO(Jerome): queryPairs not yet released ... but does more or less the
  // above agent_index->queryPairs([this](Agent * a1, Agent * a2) {
  // resolve_collision(a1, a2); });
}

#if 0
void World::update_agent_ghosts() {
  ghost_envelops.clear();
  ghosts.clear();
  const auto grid = get_lattice_grid();
  ghost_index =
      std::make_shared<geos::index::strtree::TemplateSTRtree<Ghost *>>(
          agents.size() * grid.size());
  for (const auto &delta : grid) {
    for (const auto &agent : agents) {
      Ghost ghost(agent.get());
      ghost.position += delta;
      auto bb = envelop(ghost.position, ghost.radius);
      ghosts.push_back(ghost);
      ghost_envelops.push_back(bb);
      ghost_index->insert(&ghost_envelops.back(), &ghosts.back());
    }
  }
}
#endif

void World::update_agents_strtree() {
  agent_envelops.clear();
  agent_index =
      std::make_shared<geos::index::strtree::TemplateSTRtree<Agent *>>(
          agents.size());
  for (const auto &agent : agents) {
    auto &bb =
        agent_envelops.emplace_back(agent->pose.position[0] - agent->radius,
                                    agent->pose.position[0] + agent->radius,
                                    agent->pose.position[1] - agent->radius,
                                    agent->pose.position[1] + agent->radius);
    agent_index->insert(&bb, (void *)(agent.get()));
  }

  agents_strtree_is_updated = true;

  // if (has_lattice) {
  //   update_agent_ghosts();
  // }
}

void World::check_static_strtree() {
  if (!static_strtree_is_updated) {
    update_static_strtree();
  }
}

void World::check_agents_strtree() {
  if (!agents_strtree_is_updated) {
    update_agents_strtree();
  }
}

void World::update_static_strtree() {
  static_envelops.clear();
  obstacles_index =
      std::make_shared<geos::index::strtree::TemplateSTRtree<Obstacle *>>(
          obstacles.size());
  walls_index = std::make_shared<geos::index::strtree::TemplateSTRtree<Wall *>>(
      walls.size());
  // TODO(J): should coordinates be ordered?
  for (const auto &wall : walls) {
    const Vector2 &p1 = wall->line.p1;
    const Vector2 &p2 = wall->line.p2;
    auto &bb = static_envelops.emplace_back(p1[0], p2[0], p1[1], p2[1]);
    walls_index->insert(&bb, (void *)(wall.get()));
  }
  for (const auto &obstacle : obstacles) {
    const Vector2 &p = obstacle->disc.position;
    const ng_float_t r = obstacle->disc.radius;
    auto &bb =
        static_envelops.emplace_back(p[0] - r, p[0] + r, p[1] - r, p[1] + r);
    obstacles_index->insert(&bb, (void *)(obstacle.get()));
  }
  static_strtree_is_updated = true;
}

ng_float_t World::compute_safety_violation(const Agent *agent,
                                           std::optional<float> safety_margin) {
  float sm = 0.0;
  if (!safety_margin) {
    if (Behavior *b = agent->get_behavior()) {
      sm = b->get_safety_margin();
    }
  } else {
    sm = *safety_margin;
  }
  if (sm <= 0) {
    return 0;
  }
  check_agents_strtree();
  check_static_strtree();
  const ng_float_t radius = agent->radius + sm;
  const auto p = agent->pose.position;
  const BoundingBox bb{p[0] - radius, p[0] + radius, p[1] - radius,
                       p[1] + radius};
  ng_float_t d = 0;
  for (const auto &[_delta, lbb] : subdivide_bounding_box(bb)) {
    const Vector2 &delta = _delta;
    agent_index->query(lbb, [&d, &p, &radius, &delta, &agent](Agent *other) {
      if (other != agent) {
        d = std::max(
            penetration_inside_disc(
                Disc{other->pose.position + delta, other->radius}, p, radius),
            d);
      }
    });
    obstacles_index->query(lbb, [&d, &p, &radius, &delta](Obstacle *o) {
      d = std::max(
          penetration_inside_disc(o->get_translated_disc(delta), p, radius), d);
    });
  }
  walls_index->query(bb, [&d, &p, &radius](Wall *w) {
    d = std::max(penetration_inside_line(w->line, p, radius), d);
  });
  // if (has_lattice) {
  //   ghost_index->query(bb, [&d, &p, &radius, &agent](Ghost *other) {
  //     if (agent->uid != other->uid) {
  //       d = std::max(penetration_inside_disc(*other, p, radius), d);
  //     }
  //   });
  // }
  return d;
}

bool World::agents_are_idle() const {
  return std::all_of(agents.cbegin(), agents.cend(),
                     [](auto a) { return a->idle(); });
}

bool World::agents_are_idle_or_stuck() const {
  return std::all_of(agents.cbegin(), agents.cend(), [this](auto a) {
    return a->idle() || a->has_been_stuck_since(time - 1.0);
  });
}

// TODO(Jerome) (extend to lattice)
bool World::space_agents_apart_once(ng_float_t minimal_distance,
                                    bool with_safety_margin) {
  bool r = false;
  for (const auto &agent : agents) {
    agent->collision_correction = Vector2::Zero();
  }
  for (const auto &agent : agents) {
    Agent *a1 = agent.get();
    ng_float_t margin = minimal_distance;
    if (with_safety_margin && agent->get_behavior()) {
      margin += agent->get_behavior()->get_safety_margin();
    }
    const auto p = agent->pose.position;
    const ng_float_t radius = agent->radius + margin;
    const BoundingBox bb{p[0] - radius, p[0] + radius, p[1] - radius,
                         p[1] + radius};

    for (const auto &[d, lbb] : subdivide_bounding_box(bb)) {
      const Vector2 &delta = d;
      agent_index->query(lbb, [this, &r, &a1, &delta, margin](Agent *a2) {
        if (a1->uid < a2->uid) {
          r |= resolve_collision(a1, a2, delta, margin);
        }
      });
      obstacles_index->query(lbb, [this, &r, &a1, &delta, margin](Obstacle *o) {
        r |= resolve_collision(a1, &(o->disc), delta, margin);
      });
    }

    walls_index->query(bb, [this, &r, &a1, margin](Wall *w) {
      r |= resolve_collision(a1, &(w->line), margin);
    });
  }
  for (const auto &agent : agents) {
    agent->pose.position += agent->collision_correction;
  }
  if (_has_lattice) {
    wrap_agents_on_lattice();
  }
  return r;
}

void World::space_agents_apart(ng_float_t minimal_distance,
                               bool with_safety_margin,
                               unsigned max_iterations) {
  if (_has_lattice) {
    wrap_agents_on_lattice();
  }
  update_static_strtree();
  update_agents_strtree();
  for (unsigned i = 0; i < max_iterations; ++i) {
    if (!space_agents_apart_once(minimal_distance, with_safety_margin)) {
      return;
    }
    update_agents_strtree();
  }
}

World::Lattice World::get_lattice(unsigned index) const {
  if (index < lattice.size()) {
    return lattice[index];
  }
  return std::nullopt;
}

void World::set_lattice(unsigned index, const World::Lattice &value) {
  if (index < lattice.size()) {
    lattice[index] = value;
    _has_lattice = (lattice[0] || lattice[1]);
  }
}

std::vector<Vector2> World::get_lattice_grid(bool include_zero, bool c8) const {
  std::vector<Vector2> rs;
  if (lattice[0] && lattice[1]) {
    auto l0 = std::get<1>(*(lattice[0]));
    auto l1 = std::get<1>(*(lattice[1]));
    if (c8) {
      rs = {{-l0, -l1}, {-l0, 0},  {-l0, l1}, {0, -l1},
            {0, l1},    {l0, -l1}, {l0, 0},   {l0, l1}};
    } else {
      rs = {{-l0, 0}, {0, -l1}, {0, l1}, {l0, 0}};
    }
  } else if (lattice[0]) {
    auto l0 = std::get<1>(*(lattice[0]));
    rs = {{-l0, 0}, {l0, 0}};
  } else if (lattice[1]) {
    auto l1 = std::get<1>(*(lattice[1]));
    rs = {{0, -l1}, {0, l1}};
  }
  if (include_zero) {
    rs.push_back(Vector2());
  }
  return rs;
}

void World::wrap_agents_on_lattice() {
  for (int i = 0; i < 2; ++i) {
    if (lattice[i]) {
      auto [from, length] = *(lattice[i]);
      for (const auto &agent : agents) {
        ng_float_t c = agent->pose.position[i];
        if (wrap_lattice(c, from, length)) {
          agent->pose.position[i] = c;
        }
      }
    }
  }
}

BoundingBox World::get_minimal_bounding_box() const {
  std::vector<ng_float_t> xs;
  std::vector<ng_float_t> ys;
  for (const auto &agent : agents) {
    xs.push_back(agent->pose.position[0] - agent->radius);
    xs.push_back(agent->pose.position[0] + agent->radius);
    ys.push_back(agent->pose.position[1] - agent->radius);
    ys.push_back(agent->pose.position[1] + agent->radius);
  }
  for (const auto &o : obstacles) {
    xs.push_back(o->disc.position[0] - o->disc.radius);
    xs.push_back(o->disc.position[0] + o->disc.radius);
    ys.push_back(o->disc.position[1] - o->disc.radius);
    ys.push_back(o->disc.position[1] + o->disc.radius);
  }
  for (const auto &w : walls) {
    xs.push_back(w->line.p1[0]);
    xs.push_back(w->line.p2[0]);
    ys.push_back(w->line.p1[1]);
    ys.push_back(w->line.p2[1]);
  }
  if (xs.size() == 0 || ys.size() == 0) {
    return {0, 0, 0, 0};
  }
  return {*std::min_element(xs.begin(), xs.end()),
          *std::max_element(xs.begin(), xs.end()),
          *std::min_element(ys.begin(), ys.end()),
          *std::max_element(ys.begin(), ys.end())};
}

void World::add_random_obstacles(unsigned number, ng_float_t min_radius,
                                 ng_float_t max_radius, ng_float_t margin,
                                 unsigned max_tries) {
  std::vector<Disc> others = get_discs(true);
  ng_float_t gap = 0.0;
  for (const auto &agent : get_agents()) {
    auto r = agent->radius;
    if (agent->behavior) {
      r += agent->behavior->get_safety_margin();
    }
    others.emplace_back(agent->pose.position, r);
    gap = std::max(gap, 2 * r);
  }
  gap += margin;
  auto discs = sample_discs(get_random_generator(), number, get_bounding_box(),
                            min_radius, max_radius, gap, margin, others,
                            max_tries, get_lattice_grid());
  for (const auto &disc : discs) {
    add_obstacle(disc);
  }
}

}  // namespace navground::sim
