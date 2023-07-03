/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/world.h"

#include "navground/sim/sampling/sampler.h"

using navground::core::Frame;

namespace navground::sim {

void World::set_seed(unsigned seed) { set_random_seed(seed); }

static bool wrap_lattice(float &x, float from, float length) {
  const float delta = x - from;
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

static BoundingBox envelop(const Vector2 &position, float radius) {
  return {position[0] - radius, position[0] + radius, position[1] - radius,
          position[1] + radius};
}

std::optional<Vector2> penetration_vector_inside_line(const LineSegment &line,
                                                      const Vector2 &center,
                                                      float radius) {
  float y = (center - line.p1).dot(line.e2);
  if (abs(y) < radius) {
    float x = (center - line.p1).dot(line.e1);
    if (x < radius + 1e-3 || x > line.length - radius - 1e-3)
      return std::nullopt;
    float p = radius - abs(y);
    if (y < 0) p *= -1;
    return p * line.e2;
  }
  return std::nullopt;
}

float penetration_inside_line(const LineSegment &line, const Vector2 &center,
                              float radius) {
  float y = (center - line.p1).dot(line.e2);
  if (abs(y) < radius) {
    float x = (center - line.p1).dot(line.e1);
    if (x < radius + 1e-3 || x > line.length - radius - 1e-3) return 0.0;
    return radius - abs(y);
  }
  return 0.0f;
}

float penetration_inside_disc(const Disc &disc, const Vector2 &center,
                              float radius) {
  return std::max(0.0f, radius + disc.radius - (disc.position - center).norm());
}

void World::update(float time_step) {
  if (!ready) {
    prepare();
    ready = true;
  }
  for (auto &a : agents) {
    a->update(time_step, time, this);
  }
  for (auto &a : agents) {
    a->actuate(time_step);
  }
  update_agents_strtree();
  update_collisions();
  if (has_lattice) {
    wrap_agents_on_lattice();
  }
  // bool r = false;

  // if (r) {
  //   update_agents_strtree();
  // }
  time += time_step;
  step++;
}

void World::update_dry(float time_step) {
  if (!ready) {
    prepare();
    ready = true;
  } else {
    update_agents_strtree();
  }
  for (auto &a : agents) {
    a->update(time_step, time, this);
  }
  time += time_step;
  step++;
}

void World::add_entity(Entity *entity) { entities[entity->uid] = entity; }

void World::add_agent(const std::shared_ptr<Agent> &agent) {
  if (agent) {
    if (entities.count(agent->uid) == 0) {
      agents.push_back(agent);
      ready = false;
      add_entity(agent.get());
    } else {
      std::cerr << "This agent was already added!" << std::endl;
    }
  }
}

void World::add_wall(const LineSegment &wall) {
  walls.push_back(wall);
  add_entity(&walls.back());
  ready = false;
}

void World::add_wall(const Wall &wall) {
  if (entities.count(wall.uid) == 0) {
    walls.push_back(wall);
    add_entity(&walls.back());
    ready = false;
  } else {
    std::cerr << "This wall was already added!" << std::endl;
  }
}

void World::add_obstacle(const Disc &obstacle) {
  obstacles.push_back(obstacle);
  add_entity(&obstacles.back());
  ready = false;
}

void World::add_obstacle(const Obstacle &obstacle) {
  if (entities.count(obstacle.uid) == 0) {
    obstacles.push_back(obstacle);
    add_entity(&obstacles.back());
    ready = false;
  } else {
    std::cerr << "This obstacle was already added!" << std::endl;
  }
}


void World::set_obstacles(const std::vector<Disc> &values) {
  obstacles.clear();
  for (const auto &value : values) {
    add_obstacle(value);
  }
}

// TODO(jerome) (clear)
void World::set_walls(const std::vector<LineSegment> &values) {
  walls.clear();
  for (const auto &value : values) {
    add_wall(value);
  }
}

void World::prepare() {
  // TODO(Jerome) Should only execute it once if not already prepared.
  update_static_strtree();
  update_agents_strtree();

  for (auto &a : agents) {
    if (a->state_estimation) {
      // a->state_estimation->world = this;
      a->state_estimation->prepare(a.get(), this);
    }
    a->collision_correction = Vector2::Zero();
    if (a->behavior) {
      a->behavior->set_kinematics(a->kinematics);
      a->behavior->set_radius(a->radius);
      a->controller.set_behavior(a->behavior);
      // TODO(J): should be optional now that we added support for external run-loops
      a->controller.set_cmd_frame(Frame::absolute);
    }
  }
  ready = true;
}

void World::run(unsigned steps, float time_step) {
  for (size_t i = 0; i < steps; i++) {
    update(time_step);
  }
}

const std::vector<std::shared_ptr<Agent>> &World::get_agents() const {
  return agents;
}

std::vector<Agent *> World::get_agents_in_region(const BoundingBox &bb) const {
  std::vector<Agent *> rs;
  // std::transform(agents.cbegin(), agents.cend(), std::back_inserter(rs),
  // [](const auto & a) { return &a; });
  agent_index->query(bb, rs);
  return rs;
}

// TODO(Jerome): add agent/obstacle radius to narrow phase
std::vector<Neighbor> World::get_neighbors(const Agent *agent,
                                           float distance) const {
  std::vector<Neighbor> rs;
  const auto bb = envelop(agent->pose.position, distance);
  const unsigned n = agents.size() + (has_lattice ? ghosts.size() : 0);
  rs.reserve(n);
  const auto p = agent->pose.position;
  const auto uid = agent->uid;
  if (has_lattice) {
    ghost_index->query(bb, [&rs, &uid, &p, &distance](Ghost *g) {
      if (g->uid != uid && (g->position - p).norm() < distance + g->radius) {
        rs.push_back(*g);
      }
    });
  }
  agent_index->query(bb, [&rs, &agent, &p, &distance](Agent *a) {
    if (a != agent && (a->pose.position - p).norm() < distance + a->radius) {
      rs.push_back(a->as_neighbor());
    }
  });
  return rs;
}

// TODO(Jerome) Should cache .. this needs to be fast
std::vector<Disc> World::get_discs() const {
  std::vector<Disc> discs(obstacles.size());
  std::transform(obstacles.cbegin(), obstacles.cend(), discs.begin(),
                 [](const auto &o) { return o.disc; });
  return discs;
}

// TODO(Jerome) Should cache .. this needs to be fast
const std::vector<Obstacle> &World::get_obstacles() const { return obstacles; }

// TODO(J): complete
std::vector<Disc *> World::get_static_obstacles_in_region(
    [[maybe_unused]] const BoundingBox &bb) const {
  return {};
}

// TODO(Jerome) Should cache .. this needs to be fast
const std::vector<Wall> &World::get_walls() const { return walls; }

// TODO(Jerome) Should cache .. this needs to be fast
std::vector<LineSegment> World::get_line_obstacles() const {
  std::vector<LineSegment> lines(walls.size());
  std::transform(walls.cbegin(), walls.cend(), lines.begin(),
                 [](const auto &w) { return w.line; });
  return lines;
}

// TODO(J): complete
std::vector<LineSegment *> World::get_line_obstacles_in_region(
    [[maybe_unused]] const BoundingBox &bb) const {
  return {};
}

bool World::resolve_collision(Agent *a1, Agent *a2, float margin) {
  auto delta = a1->pose.position - a2->pose.position;
  float p = delta.norm() - a1->radius - a2->radius - margin;
  if (p > 0) {
    return false;
  }
  auto u = delta / delta.norm();
  auto correction = (-p * 0.5 + 1e-3) * u;
  a1->collision_correction += correction;
  a2->collision_correction -= correction;
  float d = a1->twist.velocity.dot(-u);
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
bool World::resolve_collision(Agent *agent, Disc *disc, float margin) {
  auto delta = agent->pose.position - disc->position;
  float p = delta.norm() - agent->radius - disc->radius - margin;
  if (p > 0) {
    return false;
  }
  auto u = delta / delta.norm();
  auto correction = (-p * 1.0 + 1e-3) * u;
  agent->collision_correction += correction;
  float d = agent->twist.velocity.dot(-u);
  if (d > 0) {
    agent->twist.velocity += d * u;
  }
  return true;
}

bool World::resolve_collision(Agent *agent, LineSegment *line, float margin) {
  if (auto p = penetration_vector_inside_line(*line, agent->pose.position,
                                              agent->radius + margin)) {
    auto n = p->norm();
    auto u = *p / n;
    agent->collision_correction = (n + 1e-3) * u;
    float d = agent->twist.velocity.dot(u);
    if (d > 0) {
      agent->twist.velocity += d * u;
    }
    return true;
  }
  return false;
}

void World::record_collision(const Entity *e1, const Entity *e2) {
  collisions.emplace(e1, e2);
}

bool World::in_collision(const Entity *e1, const Entity *e2) const {
  return collisions.count({e1, e2}) > 0 || collisions.count({e2, e1}) > 0;
}

// TODO(J): spare recomputing the envelops?
void World::udpate_agent_collisions(Agent *a1) {
  // const BoundingBox &bb = agent_envelops[i];
  const BoundingBox bb = envelop(a1->pose.position, a1->radius);
  agent_index->query(bb, [this, a1](Agent *a2) {
    if (a1 < a2) {
      if (resolve_collision(a1, a2)) {
        record_collision(a1, a2);
      }
    }
  });
  obstacles_index->query(bb, [this, a1](Obstacle *o) {
    if (resolve_collision(a1, &(o->disc))) {
      record_collision(a1, o);
    }
  });
  walls_index->query(bb, [this, a1](Wall *w) {
    if (resolve_collision(a1, &(w->line))) {
      record_collision(a1, w);
    }
  });
  if (has_lattice) {
    ghost_index->query(bb, [this, a1](Ghost *a2) {
      if (resolve_collision(a1, a2)) {
        record_collision(a1, a2);
      }
    });
  }
}

void World::update_collisions() {
  collisions.clear();
  for (const auto &agent : agents) {
    udpate_agent_collisions(agent.get());
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

void World::update_agent_ghosts() {
  ghost_envelops.clear();
  ghosts.clear();
  const auto grid = lattice_grid();
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

  if (has_lattice) {
    update_agent_ghosts();
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
    const Vector2 &p1 = wall.line.p1;
    const Vector2 &p2 = wall.line.p2;
    auto &bb = static_envelops.emplace_back(p1[0], p2[0], p1[1], p2[1]);
    walls_index->insert(&bb, (void *)(&wall));
  }
  for (const auto &obstacle : obstacles) {
    const Vector2 &p = obstacle.disc.position;
    const float r = obstacle.disc.radius;
    auto &bb =
        static_envelops.emplace_back(p[0] - r, p[0] + r, p[1] - r, p[1] + r);
    obstacles_index->insert(&bb, (void *)(&obstacle));
  }
}

float World::compute_safety_violation(const Agent *agent) const {
  if (Behavior *b = agent->get_behavior()) {
    const float safety_margin = b->get_safety_margin();
    if (safety_margin == 0) return 0.0f;
    const float radius = agent->radius + safety_margin;
    const auto p = agent->pose.position;
    const BoundingBox bb{p[0] - radius, p[0] + radius, p[1] - radius,
                         p[1] + radius};
    float d = 0.0f;
    agent_index->query(bb, [&d, &p, &radius, &agent](Agent *other) {
      if (agent != other) {
        d = std::max(penetration_inside_disc(
                         Disc{other->pose.position, other->radius}, p, radius),
                     d);
      }
    });
    obstacles_index->query(bb, [&d, &p, &radius](Obstacle *o) {
      d = std::max(penetration_inside_disc(o->disc, p, radius), d);
    });
    walls_index->query(bb, [&d, &p, &radius](Wall *w) {
      d = std::max(penetration_inside_line(w->line, p, radius), d);
    });
    if (has_lattice) {
      ghost_index->query(bb, [&d, &p, &radius, &agent](Ghost *other) {
        if (agent->uid != other->uid) {
          d = std::max(penetration_inside_disc(*other, p, radius), d);
        }
      });
    }
    return d;
  }
  return 0.0f;
}

bool World::agents_are_idle() const {
  return std::all_of(agents.cbegin(), agents.cend(),
                     [](auto a) { return a->idle(); });
}

// TODO(Jerome) (extend to lattice)
bool World::space_agents_apart_once(float minimal_distance,
                                    bool with_safety_margin) {
  bool r = false;
  for (const auto &agent : agents) {
    agent->collision_correction = Vector2::Zero();
  }
  for (const auto &agent : agents) {
    Agent *a1 = agent.get();
    float margin = minimal_distance;
    if (with_safety_margin && agent->get_behavior()) {
      margin += agent->get_behavior()->get_safety_margin();
    }
    const auto p = agent->pose.position;
    const float radius = agent->radius + margin;
    const BoundingBox bb{p[0] - radius, p[0] + radius, p[1] - radius,
                         p[1] + radius};
    agent_index->query(bb, [this, &r, &a1, margin](Agent *a2) {
      if (a1 < a2) {
        r |= resolve_collision(a1, a2, margin);
      }
    });
    obstacles_index->query(bb, [this, &r, &a1, margin](Obstacle *o) {
      r |= resolve_collision(a1, &(o->disc), margin);
    });
    walls_index->query(bb, [this, &r, &a1, margin](Wall *w) {
      r |= resolve_collision(a1, &(w->line), margin);
    });
  }
  for (const auto &agent : agents) {
    agent->pose.position += agent->collision_correction;
  }
  return r;
}

void World::space_agents_apart(float minimal_distance, bool with_safety_margin,
                               unsigned max_iterations) {
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
    has_lattice = (lattice[0] || lattice[1]);
  }
}

std::vector<Vector2> World::lattice_grid() const {
  if (lattice[0] && lattice[1]) {
    auto l0 = std::get<1>(*(lattice[0]));
    auto l1 = std::get<1>(*(lattice[1]));
    return {{-l0, -l1}, {-l0, 0},  {-l0, l1}, {0, -l1},
            {0, l1},    {l0, -l1}, {l0, 0},   {l0, l1}};
  }
  if (lattice[0]) {
    auto l0 = std::get<1>(*(lattice[0]));
    return {{-l0, 0}, {l0, 0}};
  }
  if (lattice[1]) {
    auto l1 = std::get<1>(*(lattice[1]));
    return {{0, -l1}, {0, l1}};
  }
  return {};
}

void World::wrap_agents_on_lattice() {
  for (int i = 0; i < 2; ++i) {
    if (lattice[i]) {
      auto [from, length] = *(lattice[i]);
      for (const auto &agent : agents) {
        float c = agent->pose.position[i];
        if (wrap_lattice(c, from, length)) {
          agent->pose.position[i] = c;
        }
      }
    }
  }
}

}  // namespace navground::sim
