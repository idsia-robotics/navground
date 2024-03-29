#include "navground/sim/experimental_run.h"

#include <highfive/H5DataSpace.hpp>
#include <string>

#include "navground/core/yaml/yaml.h"
#include "navground/sim/yaml/world.h"

using namespace navground::sim;

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

class TimeProbe : public RecordProbe {
 public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    data->push(run->get_world()->get_time());
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
      data->push(pose.position[0]);
      data->push(pose.position[1]);
      data->push(pose.orientation);
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
      data->push(twist.velocity[0]);
      data->push(twist.velocity[1]);
      data->push(twist.angular_speed);
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
      data->push(twist.velocity[0]);
      data->push(twist.velocity[1]);
      data->push(twist.angular_speed);
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size(), 3};
  }
};

class TargetProbe : public RecordProbe {
 public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      if (auto b = agent->get_behavior()) {
        const auto target = b->get_target();
        const auto position = target.position.value_or(Vector2::Zero());
        data->push(position[0]);
        data->push(position[1]);
        data->push(target.orientation.value_or(0.0));
      } else {
        data->push(0);
        data->push(0);
        data->push(0);
      }
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size(), 3};
  }
};

class SafetyViolationProbe : public RecordProbe {
 public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    const auto world = run->get_world();
    for (const auto &agent : world->get_agents()) {
      data->push(world->compute_safety_violation(agent.get()));
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
      data->push(world->get_step());
      data->push(e1->uid);
      data->push(e2->uid);
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
      data->push(agent->get_time_since_stuck());
    }
  }

  Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size()};
  }
};

class EfficacyProbe : public RecordProbe {
 public:
  using RecordProbe::RecordProbe;
  using Type = ng_float_t;

  void update(ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      const auto b = agent->get_behavior();
      data->push(b ? b->get_efficacy() : 1);
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
    for (const auto &agent : run->get_world()->get_agents()) {
      if (Task *task = agent->get_task()) {
        task->add_callback([this, &agent](const std::vector<ng_float_t> &data) {
          auto ds = get_data(std::to_string(agent->uid));
          ds->append(data);
        });
      }
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

  std::map<std::string, Dataset::Shape> get_shapes(
      const World &world) const override {
    std::map<std::string, Dataset::Shape> shapes;
    for (const auto &agent : world.get_agents()) {
      if (Task *task = agent->get_task()) {
        shapes[std::to_string(agent->uid)] = {task->get_log_size()};
      }
    }
    return shapes;
  }
};

void ExperimentalRun::prepare() {
  _world_yaml = YAML::dump<World>(_world.get());
  _number = _world->get_agents().size();
  const auto &agents = _world->get_agents();
  for (unsigned i = 0; i < _number; ++i) {
    _indices[agents[i].get()] = i;
  }
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
  if (_state != State::init) return;
  start();
  for (unsigned step = 0; step < _run_config.steps; step++) {
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
  if (_state != State::running || _steps > _run_config.steps) return;
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
  store_world(_world_yaml, group);
  store_attribute<ng_float_t>(_run_config.time_step, "time_step", group);
  store_attribute<unsigned>(_run_config.steps, "maximal_steps", group);
  store_attribute<unsigned>(_steps, "steps", group);
  store_attribute<unsigned>(get_seed(), "seed", group);
  store_attribute<ng_float_t>(_world->get_time(), "final_sim_time", group);
  const unsigned long d = get_duration_ns().count();
  group
      .createAttribute<unsigned long>("duration_ns",
                                      HighFive::DataSpace::From(d))
      .write(d);

  for (auto &[key, ds] : _records) {
    ds->save(key, group);
  }
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

static std::map<std::string, std::string> find_full_names(
    const std::set<std::string> &all, const std::string &begin) {
  std::map<std::string, std::string> rs;
  for (const auto &value : all) {
    auto r = value.find(begin);
    if (r == 0) {
      rs.emplace(value.substr(begin.size()), value);
    }
  }
  return rs;
}

static std::map<std::string, std::string> find_full_names(
    const std::set<std::string> &all) {
  std::map<std::string, std::string> rs;
  for (const auto &value : all) {
    rs.emplace(value, value);
  }
  return rs;
}

std::set<std::string> ExperimentalRun::get_record_names(
    const std::string &group) const {
  if (group.empty()) {
    return _record_names;
  } else {
    return find_names(_record_names, group + "/");
  }
}

std::map<std::string, std::string> ExperimentalRun::get_group(
    const std::string &name) const {
  if (name.empty()) {
    return find_full_names(_record_names);
  } else {
    return find_full_names(_record_names, name + "/");
  }
}
