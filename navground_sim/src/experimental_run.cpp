#include "navground/sim/experimental_run.h"

#include <highfive/H5DataSpace.hpp>
#include <limits>
#include <string>

#include "navground/core/yaml/yaml.h"
#include "navground/sim/yaml/world.h"

namespace navground::sim {

#if 0
static void store_world(const World &world, HighFive::Group &group) {
  const std::string yaml = YAML::dump<World>(&world);
  group.createAttribute<std::string>("world", HighFive::DataSpace::From(yaml))
      .write(yaml);
}
#endif

static void store_world(const std::string &yaml, HighFive::Group &group) {
  group.createAttribute<std::string>("world", HighFive::DataSpace::From(yaml))
      .write(yaml);
}

template <typename T>
static void store_attribute(T value, const std::string &name,
                            HighFive::Group &group) {
  group.createAttribute<T>(name, HighFive::DataSpace::From(value)).write(value);
}

std::shared_ptr<Dataset>
extract_collision_events(const std::shared_ptr<Dataset> &collisions,
                         unsigned min_interval) {
  const auto shape = collisions->get_shape();
  if (shape.size() != 2 && shape[1] != 3) {
    std::cerr << "Collisions dataset has unexpected shape" << std::endl;
    return nullptr;
  }
  const auto ptr = collisions->get_typed_data<unsigned>();
  if (!ptr) {
    std::cerr << "Collisions dataset has unexpected type" << std::endl;
    return nullptr;
  }
  std::shared_ptr<Dataset> collision_events = Dataset::make<unsigned>({4});
  collision_events->set_dtype<unsigned>();
  const std::vector<unsigned> values = *ptr;
  // (e1, e2) -> (begin, end)
  std::map<std::tuple<unsigned, unsigned>, std::tuple<unsigned, unsigned>> ts;
  for (size_t i = 0; i < shape[0]; ++i) {
    const unsigned t = values[3 * i];
    const std::tuple<unsigned, unsigned> es{values[3 * i + 1],
                                            values[3 * i + 2]};
    if (!ts.count(es)) {
      ts[es] = {t, t};
    } else {
      auto &vs = ts[es];
      if (t > std::get<1>(vs) + min_interval) {
        collision_events->push<unsigned>(std::get<0>(vs));
        collision_events->push<unsigned>(std::get<1>(vs));
        collision_events->push<unsigned>(std::get<0>(es));
        collision_events->push<unsigned>(std::get<1>(es));
        ts[es] = {t, t};
      } else {
        std::get<1>(vs) = t;
      }
    }
  }
  for (const auto &[es, vs] : ts) {
    collision_events->push<unsigned>(std::get<0>(vs));
    collision_events->push<unsigned>(std::get<1>(vs));
    collision_events->push<unsigned>(std::get<0>(es));
    collision_events->push<unsigned>(std::get<1>(es));
  }
  return collision_events;
}

std::shared_ptr<Dataset> extract_steps_to_collision(
    unsigned first_agent_id, unsigned last_agent_id, unsigned steps,
    const std::shared_ptr<Dataset> &collisions, unsigned min_interval) {
  const size_t n = last_agent_id - first_agent_id + 1;
  const unsigned m = std::numeric_limits<unsigned>::max();
  std::vector<unsigned> data(n * steps, m);
  std::shared_ptr<Dataset> ds = Dataset::make<unsigned>({n});
  ds->set_data(data);
  const auto collision_events_ds =
      extract_collision_events(collisions, min_interval);
  auto vs = ds->as_tensor<unsigned, 2>();
  const auto events = collision_events_ds->as_tensor<unsigned, 2>();
  for (int j = 0; j < events.dimension(1); j++) {
    for (size_t i = events(0, j); i <= events(1, j); i++) {
      vs(events(2, j) - first_agent_id, i) = 0;
      vs(events(3, j) - first_agent_id, i) = 0;
    }
  }
  for (int i = vs.dimension(1) - 2; i >= 0; i--) {
    for (int j = 0; j < vs.dimension(0); j++) {
      if (vs(j, i) > 0 && vs(j, i + 1) < m) {
        vs(j, i) = vs(j, i + 1) + 1;
      }
    }
  }
  return ds;
}

class TimeProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    get_data()->push(run->get_world()->get_time());
  }

  Dataset::Shape get_shape(const World &world) const override { return {}; }
};

class PoseProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      const auto pose = agent->pose;
      get_data()->push(pose.position[0]);
      get_data()->push(pose.position[1]);
      get_data()->push(pose.orientation);
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size(), 3};
  }
};

class TwistProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      const auto twist = agent->twist;
      get_data()->push(twist.velocity[0]);
      get_data()->push(twist.velocity[1]);
      get_data()->push(twist.angular_speed);
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size(), 3};
  }
};

class CmdProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      const auto twist = agent->last_cmd;
      get_data()->push(twist.velocity[0]);
      get_data()->push(twist.velocity[1]);
      get_data()->push(twist.angular_speed);
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size(), 3};
  }
};

class ActuatedCmdProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      const auto twist = agent->get_actuated_cmd();
      get_data()->push(twist.velocity[0]);
      get_data()->push(twist.velocity[1]);
      get_data()->push(twist.angular_speed);
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size(), 3};
  }
};

core::Target
ExperimentalRun::target_from_data(const std::vector<ng_float_t> &data) {
  core::Target target;
  if (data[0]) {
    target.position = Vector2(data[1], data[2]);
  }
  if (data[3]) {
    target.orientation = data[4];
  }
  if (data[5]) {
    target.speed = data[6];
  }
  if (data[7]) {
    target.direction = Vector2(data[8], data[9]);
  }
  if (data[10]) {
    target.angular_speed = data[11];
  }
  target.position_tolerance = data[12];
  target.orientation_tolerance = data[13];
  return target;
}

std::vector<ng_float_t>
ExperimentalRun::data_from_target(const core::Target &target) {
  const auto position = target.position.value_or(Vector2::Zero());
  const auto direction = target.direction.value_or(Vector2::Zero());
  return std::vector<ng_float_t>{
      static_cast<ng_float_t>(bool(target.position)),
      position[0],
      position[1],
      static_cast<ng_float_t>(bool(target.orientation)),
      target.orientation.value_or(0),
      static_cast<ng_float_t>(bool(target.speed)),
      target.speed.value_or(0),
      static_cast<ng_float_t>(bool(target.direction)),
      direction[0],
      direction[1],
      static_cast<ng_float_t>(bool(target.angular_speed)),
      target.angular_speed.value_or(0),
      target.position_tolerance,
      target.orientation_tolerance};
}

class TargetProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      if (auto b = agent->get_behavior()) {
        get_data()->append(ExperimentalRun::data_from_target(b->get_target()));
      } else {
        get_data()->append(std::vector<Type>(0, 14));
      }
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size(), 14};
  }
};

class SafetyViolationProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    const auto world = run->get_world();
    for (const auto &agent : world->get_agents()) {
      get_data()->push(world->compute_safety_violation(agent.get()));
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size()};
  }
};

class CollisionProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = unsigned;

  void update(ExperimentalRun *run) override {
    const auto world = run->get_world();
    for (const auto &[e1, e2] : world->get_collisions()) {
      get_data()->push(world->get_step());
      get_data()->push(e1->uid);
      get_data()->push(e2->uid);
    }
  }

  Dataset::Shape get_shape(const World &world) const override { return {3}; }
};

class DeadlockProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void finalize(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      get_data()->push(agent->get_time_since_stuck());
    }
  }

  Dataset::Shape get_shape(const World &world) const override { return {}; }
};

class EfficacyProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      const auto b = agent->get_behavior();
      get_data()->push(b ? b->get_efficacy() : 1);
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size()};
  }
};

class TaskEventsProbe : public GroupRecordProbe {
public:
  using GroupRecordProbe::GroupRecordProbe;
  using Type = ng_float_t;

  void prepare(ExperimentalRun *run) override {
    GroupRecordProbe::prepare(run);
    unsigned i = 0;
    const bool use_uid = run->get_record_config().use_agent_uid_as_key;
    for (const auto &agent : run->get_world()->get_agents()) {
      if (Task *task = agent->get_task()) {
        const std::string key = std::to_string(use_uid ? agent->uid : i);
        task->add_callback([this, key](const std::vector<ng_float_t> &data) {
          get_data(key)->append(data);
        });
      }
      i++;
    }
  }

  void finalize(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      if (Task *task = agent->get_task()) {
        // TODO(Jerome): should remove just the callback we added.
        task->clear_callbacks();
      }
    }
  }

  std::map<std::string, Dataset::Shape>
  get_shapes(const World &world, bool use_agent_uid_as_key) const override {
    std::map<std::string, Dataset::Shape> shapes;
    unsigned i = 0;
    for (const auto &agent : world.get_agents()) {
      if (Task *task = agent->get_task()) {
        const std::string key =
            std::to_string(use_agent_uid_as_key ? agent->uid : i);
        shapes[key] = {task->get_log_size()};
      }
      i++;
    }
    return shapes;
  }
};

class NeighborsProbe : public RecordProbe {
public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;
  unsigned number;
  bool relative;

  void prepare(ExperimentalRun *run) override {
    const auto config = run->get_record_config().neighbors;
    if (config.number < 0) {
      number = static_cast<unsigned>(run->get_world()->get_agents().size() - 1);
    } else {
      number = config.number;
    }
    relative = config.relative;
    RecordProbe::prepare(run);
  }

  void update(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      const auto b = agent->get_behavior();
      unsigned i = 0;
      if (GeometricState *state =
              dynamic_cast<navground::core::GeometricState *>(
                  b->get_environment_state())) {
        const auto p = b->get_position();
        auto ns = state->get_neighbors();
        std::sort(ns.begin(), ns.end(),
                  [&p](const Neighbor &n1, const Neighbor &n2) {
                    return (n1.position - p).norm() < (n2.position - p).norm();
                  });
        for (auto n : ns) {
          if (i >= number) {
            break;
          }
          if (relative) {
            n = n.relative_to(b->get_pose());
          }
          get_data()->push(n.radius);
          get_data()->push(n.position[0]);
          get_data()->push(n.position[1]);
          get_data()->push(n.velocity[0]);
          get_data()->push(n.velocity[1]);
          i++;
        }
      }
      for (; i < number; ++i) {
        get_data()->append(std::vector<ng_float_t>(0, 5));
      }
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size(), number, 5};
  }
};

void ExperimentalRun::prepare() {
  if (_record_config.world) {
    _world_yaml = YAML::dump<World>(_world.get());
  } else {
    _world_yaml = "";
  }
  // const auto number = _world->get_agents().size();
  // const auto &agents = _world->get_agents();
  // for (unsigned i = 0; i < number; ++i) {
  //   _indices[agents[i].get()] = i;
  // }
  if (_record_config.time) {
    add_record_probe<TimeProbe>("times");
  }
  if (_record_config.pose) {
    add_record_probe<PoseProbe>("poses");
  }
  if (_record_config.twist) {
    add_record_probe<TwistProbe>("twists");
  }
  if (_record_config.cmd) {
    add_record_probe<CmdProbe>("cmds");
  }
  if (_record_config.actuated_cmd) {
    add_record_probe<ActuatedCmdProbe>("actuated_cmds");
  }
  if (_record_config.target) {
    add_record_probe<TargetProbe>("targets");
  }
  if (_record_config.safety_violation) {
    add_record_probe<SafetyViolationProbe>("safety_violations");
  }
  if (_record_config.collisions) {
    add_record_probe<CollisionProbe>("collisions");
  }
  if (_record_config.deadlocks) {
    add_record_probe<DeadlockProbe>("deadlocks");
  }
  if (_record_config.efficacy) {
    add_record_probe<EfficacyProbe>("efficacy");
  }
  if (_record_config.task_events) {
    add_group_record_probe<TaskEventsProbe>("task_events");
  }
  if (_record_config.neighbors.enabled) {
    add_record_probe<NeighborsProbe>("neighbors");
  }
  for (const auto &c : _record_config.sensing) {
    add_probe(
        std::make_shared<SensingProbe>(c.name, c.sensor, c.agent_indices));
  }
  for (auto &probe : _probes) {
    probe->prepare(this);
  }
}

void ExperimentalRun::start() {
  if (_state == State::init) {
    prepare();
    _begin = std::chrono::steady_clock::now();
    _state = State::running;
  }
}

void ExperimentalRun::stop() {
  if (_state == State::running) {
    _end = std::chrono::steady_clock::now();
    finalize();
    _state = State::finished;
  }
}

void ExperimentalRun::run() {
  if (_state != State::init)
    return;
  start();
  for (unsigned step = 0; step < _run_config.steps; step++) {
    if (_world->should_terminate()) {
      break;
    }
    _world->update(_run_config.time_step);
    update();
    // TODO(Jerome): Do I really need callbacks?
    // for (const auto &cb : callbacks) {
    //   cb();
    // }
    if (_run_config.terminate_when_all_idle_or_stuck &&
        _world->agents_are_idle_or_stuck()) {
      break;
    }
  }
  stop();
}

void ExperimentalRun::update() {
  if (_state != State::running || _steps > _run_config.steps)
    return;
  for (auto &probe : _probes) {
    probe->update(this);
  }
  _steps++;
}

void ExperimentalRun::finalize() {
  for (auto &probe : _probes) {
    probe->finalize(this);
  }
}

void ExperimentalRun::save(HighFive::Group &group) const {
  if (!_world_yaml.empty()) {
    store_world(_world_yaml, group);
  }
  store_attribute<ng_float_t>(_run_config.time_step, "time_step", group);
  store_attribute<unsigned>(_run_config.steps, "maximal_steps", group);
  store_attribute<unsigned>(_steps, "steps", group);
  store_attribute<unsigned>(get_seed(), "seed", group);
  store_attribute<ng_float_t>(_world->get_time(), "final_sim_time", group);
  const unsigned long d = static_cast<unsigned long>(get_duration().count());
  group
      .createAttribute<unsigned long>("duration_ns",
                                      HighFive::DataSpace::From(d))
      .write(d);

  for (auto &[key, ds] : _records) {
    ds->save(key, group);
  }
}

ng_float_t ExperimentalRun::get_final_sim_time() const {
  auto times = get_times();
  if (times) {
    return times->get_typed_data<ng_float_t>()->back();
  }
  return _steps * _run_config.time_step;
}

// {a/b, a/b/c, b}, a/ -> {b, c}
static std::set<std::string> find_names(const std::set<std::string> &all,
                                        const std::string &begin) {
  std::set<std::string> rs;
  for (const auto &value : all) {
    auto r = value.find(begin);
    if (r == 0) {
      rs.insert(value.substr(begin.size()));
    }
  }
  return rs;
}

static std::map<std::string, std::string>
find_full_names(const std::set<std::string> &all, const std::string &begin) {
  std::map<std::string, std::string> rs;
  for (const auto &value : all) {
    auto r = value.find(begin);
    if (r == 0) {
      rs.emplace(value.substr(begin.size()), value);
    }
  }
  return rs;
}

static std::map<std::string, std::string>
find_full_names(const std::set<std::string> &all) {
  std::map<std::string, std::string> rs;
  for (const auto &value : all) {
    rs.emplace(value, value);
  }
  return rs;
}

std::tuple<std::string, std::string>
ExperimentalRun::split_key(const std::string &key) {
  const auto found = key.find('/');
  if (found != std::string::npos) {
    return {key.substr(0, found), key.substr(found + 1)};
  }
  return {key, ""};
}

std::set<std::string>
ExperimentalRun::get_record_names(const std::string &group) const {
  if (group.empty()) {
    return _record_names;
  } else {
    return find_names(_record_names, group + "/");
  }
}

std::map<std::string, std::string>
ExperimentalRun::get_group(const std::string &name) const {
  if (name.empty()) {
    return find_full_names(_record_names);
  } else {
    return find_full_names(_record_names, name + "/");
  }
}

// From https://stackoverflow.com/a/42398124

template <typename T> struct SkipIt {
  using iterator_category = std::forward_iterator_tag;
  using value_type = unsigned;
  using difference_type = int;
  using pointer = unsigned *;
  using reference = unsigned &;

  SkipIt(T *t, unsigned skip) : elt(t), skip(skip) {}
  bool operator==(const SkipIt<T> &other) const { return elt == other.elt; }
  bool operator!=(const SkipIt<T> &other) const { return elt != other.elt; }
  T *operator->() const { return elt; }
  T &operator*() const { return *elt; }
  SkipIt &operator++() {
    elt += skip;
    return *this;
  }
  SkipIt operator++(int) {
    auto ret = *this;
    ++*this;
    return ret;
  }
  SkipIt operator+(int amt) const {
    auto ret = SkipIt(elt + amt * skip, skip);
    return ret;
  }

private:
  T *elt;
  unsigned skip;
};

std::set<std::tuple<Entity *, Entity *>>
ExperimentalRun::get_collisions_at_step(int step) {
  std::set<std::tuple<Entity *, Entity *>> cs;
  if (step < 0) {
    step += get_recorded_steps();
  }
  unsigned ustep = static_cast<unsigned>(step);
  if (ustep >= get_recorded_steps()) {
    return cs;
  }
  auto collisions = get_collisions();
  if (collisions) {
    auto data = *(collisions->get_typed_data<unsigned>());
    auto b = SkipIt<unsigned>(data.data(), 3);
    const unsigned number = static_cast<unsigned>(data.size() / 3);
    auto lower = std::lower_bound(b, b + number, ustep);
    unsigned i = std::distance(b, lower);
    if (i < number) {
      for (; data[3 * i] == ustep; ++i) {
        auto e1 = _world->get_entity(data[3 * i + 1]);
        auto e2 = _world->get_entity(data[3 * i + 2]);
        if (e1 && e2) {
          cs.insert({e1, e2});
        }
      }
    }
  }
  return cs;
}

void ExperimentalRun::reset() {
  _world->reset();
  go_to_step(0);
}

bool ExperimentalRun::go_to_step(int step, bool ignore_collisions,
                                 bool ignore_twists, bool ignore_cmds,
                                 bool ignore_targets, bool ignore_sensing) {
  if (step < 0) {
    step += get_recorded_steps();
  }
  if (step >= static_cast<int>(get_recorded_steps()) || step < 0) {
    return false;
  }
  _world->set_step(step);
  auto poses = get_poses();
  auto times = get_times();
  auto twists = get_twists();
  auto cmds = get_cmds();
  auto sensing = get_sensing();
  auto targets = get_targets();
  if (poses) {
    auto data = poses->as_tensor<ng_float_t, 3>();
    size_t i = 0;
    for (auto &agent : _world->get_agents()) {
      agent->pose =
          Pose2({data(0, i, step), data(1, i, step)}, data(2, i, step));
      i++;
    }
    _world->agents_moved();
  }
  if (twists && !ignore_twists) {
    auto data = twists->as_tensor<ng_float_t, 3>();
    size_t i = 0;
    for (auto &agent : _world->get_agents()) {
      agent->twist =
          Twist2({data(0, i, step), data(1, i, step)}, data(2, i, step));
      i++;
    }
  }
  if (cmds && !ignore_cmds) {
    auto data = cmds->as_tensor<ng_float_t, 3>();
    size_t i = 0;
    for (auto &agent : _world->get_agents()) {
      agent->last_cmd =
          Twist2({data(0, i, step), data(1, i, step)}, data(2, i, step));
      i++;
    }
  }
  if (targets && !ignore_targets) {
    size_t i = 0;
    auto data = targets->as_tensor<ng_float_t, 3>();
    for (auto &agent : _world->get_agents()) {
      auto behavior = agent->get_behavior();
      if (behavior) {
        behavior->set_target(target_from_data(
            std::vector<ng_float_t>(&data(0, i, step), &data(14, i, step))));
      }
      i++;
    }
  }
  if (sensing.size() && !ignore_sensing) {
    size_t i = 0;
    for (auto &agent : _world->get_agents()) {
      core::SensingState *state = dynamic_cast<core::SensingState *>(
          agent->get_behavior()->get_environment_state());
      if (state && sensing.count(i)) {
        for (auto const &[key, ds] : sensing[i]) {
          auto buffer = state->get_buffer(key);
          if (buffer) {
            ds->write_buffer(buffer, step);
          }
        }
      }
      i++;
    }
  }
  if (times) {
    auto data = times->as_tensor<ng_float_t, 1>();
    _world->set_time(data(step));
  } else {
    _world->set_time(get_time_step() * step);
  }
  if (!ignore_collisions) {
    _world->set_collisions(get_collisions_at_step(step));
  }
  return true;
}

BoundingBox ExperimentalRun::get_bounding_box() const {
  const auto &poses = get_poses();
  const auto &world = get_world();
  const auto world_bb = world->get_bounding_box();
  const auto &agents = world->get_agents();
  if (poses) {
    const auto data = poses->as_tensor<ng_float_t, 3>();
    Eigen::Tensor<ng_float_t, 2> max_over_steps =
        data.maximum(std::array<int, 1>{2});
    Eigen::Tensor<ng_float_t, 2> min_over_steps =
        data.minimum(std::array<int, 1>{2});
    size_t i = 0;
    for (const auto &agent : agents) {
      max_over_steps(0, i) += agent->radius;
      max_over_steps(1, i) += agent->radius;
      min_over_steps(0, i) -= agent->radius;
      min_over_steps(1, i) -= agent->radius;
      i++;
    }
    const Eigen::Tensor<ng_float_t, 1> max_over_agents =
        max_over_steps.maximum(std::array<int, 1>{1});
    const Eigen::Tensor<ng_float_t, 1> min_over_agents =
        min_over_steps.minimum(std::array<int, 1>{1});
    BoundingBox bb = BoundingBox(min_over_agents(0), max_over_agents(0),
                                 min_over_agents(1), max_over_agents(1));
    bb.expandToInclude(world_bb);
    return bb;
  }
  return world_bb;
}

} // namespace navground::sim
