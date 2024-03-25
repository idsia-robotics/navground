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

class TimeProbe : public Probe {
 public:
  using Probe::Probe;

  void update(const World &world) override {
    record_step();
    push(world.get_time());
  }

  Shape shape() const override { return {size()}; }
};

class PoseProbe : public Probe {
 public:
  using Probe::Probe;

  void update(const World &world) override {
    record_step();
    for (const auto &agent : world.get_agents()) {
      const auto pose = agent->pose;
      push(pose.position[0]);
      push(pose.position[1]);
      push(pose.orientation);
    }
  }

  Shape shape() const override {
    return {get_steps(), get_steps() ? size() / get_steps() / 3 : 0, 3};
  }
};

class TwistProbe : public Probe {
 public:
  using Probe::Probe;

  void update(const World &world) override {
    record_step();
    for (const auto &agent : world.get_agents()) {
      const auto twist = agent->twist;
      push(twist.velocity[0]);
      push(twist.velocity[1]);
      push(twist.angular_speed);
    }
  }

  Shape shape() const override {
    return {get_steps(), get_steps() ? size() / get_steps() / 3 : 0, 3};
  }
};

class CmdProbe : public Probe {
 public:
  using Probe::Probe;

  void update(const World &world) override {
    record_step();
    for (const auto &agent : world.get_agents()) {
      const auto twist = agent->last_cmd;
      push(twist.velocity[0]);
      push(twist.velocity[1]);
      push(twist.angular_speed);
    }
  }

  Shape shape() const override {
    return {get_steps(), get_steps() ? size() / get_steps() / 3 : 0, 3};
  }
};

class TargetProbe : public Probe {
 public:
  using Probe::Probe;

  void update(const World &world) override {
    record_step();
    for (const auto &agent : world.get_agents()) {
      if (auto b = agent->get_behavior()) {
        const auto target = b->get_target();
        const auto position = target.position.value_or(Vector2::Zero());
        push(position[0]);
        push(position[1]);
        push(target.orientation.value_or(0.0));
      } else {
        push(0);
        push(0);
        push(0);
      }
    }
  }

  Shape shape() const override {
    return {get_steps(), get_steps() ? size() / get_steps() / 3 : 0, 3};
  }
};

class SafetyViolationProbe : public Probe {
 public:
  using Probe::Probe;

  void update(const World &world) override {
    record_step();
    for (const auto &agent : world.get_agents()) {
      push(world.compute_safety_violation(agent.get()));
    }
  }

  Shape shape() const override {
    return {get_steps(), get_steps() ? size() / get_steps() : 0};
  }
};

class CollisionProbe : public Probe {
 public:
  using Probe::Probe;

  void update(const World &world) override {
    for (const auto &[e1, e2] : world.get_collisions()) {
      push(world.get_step());
      push(e1->uid);
      push(e2->uid);
    }
  }

  Shape shape() const override { return {size() / 3, 3}; }
};

class DeadlockProbe : public Probe {
 public:
  using Probe::Probe;

  void finalize(const World &world, unsigned) override {
    for (const auto &agent : world.get_agents()) {
      push(agent->get_time_since_stuck());
    }
  }

  Shape shape() const override { return {size()}; }
};

class EfficacyProbe : public Probe {
 public:
  using Probe::Probe;

  void update(const World &world) override {
    record_step();
    for (const auto &agent : world.get_agents()) {
      const auto b = agent->get_behavior();
      push(b ? b->get_efficacy() : 1);
    }
  }

  Shape shape() const override {
    return {get_steps(), get_steps() ? size() / get_steps() : 0};
  }
};

class TaskEventsProbe : public MapProbe<unsigned> {
 public:
  using MapProbe::KeyType;
  using MapProbe<unsigned>::MapProbe;

  void prepare(const World &world, unsigned) override {
    for (const auto &agent : world.get_agents()) {
      if (Task *task = agent->get_task()) {
        task->add_callback([this, &agent](const std::vector<ng_float_t> &data) {
          append(agent->uid, data);
          record_step(agent->uid);
        });
      }
    }
  }

  void finalize(const World &world, unsigned) override {
    for (const auto &agent : world.get_agents()) {
      if (Task *task = agent->get_task()) {
        // TODO(Jerome): should remove just the callback we added.
        task->clear_callbacks();
      }
    }
  }

  Shape shape(const unsigned &key) const override {
    unsigned steps = get_steps(key);
    return {steps, steps ? size(key) / steps : 0};
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
    make_probe<TimeProbe, ng_float_t>("times");
  }
  if (_record_config.pose) {
    make_probe<PoseProbe, ng_float_t>("poses");
  }
  if (_record_config.twist) {
    make_probe<TwistProbe, ng_float_t>("twists");
  }
  if (_record_config.cmd) {
    make_probe<CmdProbe, ng_float_t>("cmds");
  }
  if (_record_config.target) {
    make_probe<TargetProbe, ng_float_t>("targets");
  }
  if (_record_config.safety_violation) {
    make_probe<SafetyViolationProbe, ng_float_t>("safety_violations");
  }
  if (_record_config.collisions) {
    make_probe<CollisionProbe, unsigned>("collisions");
  }
  if (_record_config.deadlocks) {
    make_probe<DeadlockProbe, ng_float_t>("deadlocks");
  }
  if (_record_config.efficacy) {
    make_probe<EfficacyProbe, ng_float_t>("efficacy");
  }
  if (_record_config.task_events) {
    make_map_probe<TaskEventsProbe, ng_float_t>("task_events");
  }
  for (auto &[k, probe] : _probes) {
    probe->prepare(*_world, _run_config.steps);
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
  for (auto &[k, probe] : _probes) {
    probe->update(*_world);
  }
  _steps++;
}

void ExperimentalRun::finalize() {
  for (auto &[k, probe] : _probes) {
    probe->finalize(*_world, _steps);
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

  for (auto &[key, probe] : _probes) {
    probe->save(key, group);
  }
}