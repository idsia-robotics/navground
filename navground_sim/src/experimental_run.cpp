#include "navground/sim/experimental_run.h"

#include <highfive/H5DataSpace.hpp>
#include <string>

#include "navground/core/yaml/yaml.h"
#include "navground/sim/yaml/world.h"

using namespace navground::sim;

static void store_world(const World &world, HighFive::Group &group) {
  const std::string yaml = YAML::dump<World>(&world);
  group.createAttribute<std::string>("world", HighFive::DataSpace::From(yaml))
      .write(yaml);
}

static void store_world(const std::string &yaml, HighFive::Group &group) {
  group.createAttribute<std::string>("world", HighFive::DataSpace::From(yaml))
      .write(yaml);
}

template <typename T>
static void store_attribute(T value, const std::string &name,
                            HighFive::Group &group) {
  group.createAttribute<T>(name, HighFive::DataSpace::From(value)).write(value);
}

void ExperimentalRun::prepare() {
  _world_yaml = YAML::dump<World>(_world.get());
  _number = _world->get_agents().size();
  const auto &agents = _world->get_agents();
  for (unsigned i = 0; i < _number; ++i) {
    _indices[agents[i].get()] = i;
  }
  if (_record_config.time) {
    _record = true;
  }
  if (_record_config.pose) {
    _pose_data.resize(_run_config.steps * _number * 3);
    _record = true;
  }
  if (_record_config.twist) {
    _twist_data.resize(_run_config.steps * _number * 3);
    _record = true;
  }
  if (_record_config.cmd) {
    _cmd_data.resize(_run_config.steps * _number * 3);
    _record = true;
  }
  if (_record_config.target) {
    _target_data.resize(_run_config.steps * _number * 3);
    _record = true;
  }
  if (_record_config.safety_violation) {
    _safety_violation_data.resize(_run_config.steps * _number);
    _record = true;
  }
  if (_record_config.collisions) {
    _record = true;
  }
  if (_record_config.task_events) {
    _record = true;
    _task_events_data = std::vector<std::vector<float>>(_number);
    _task_events = std::vector<unsigned>(_number, 0);
    int index = 0;
    for (const auto &agent : _world->get_agents()) {
      if (Task *task = agent->get_task()) {
        auto &ds = _task_events_data[index];
        auto &n = _task_events[index];
        task->add_callback([&ds, &n](const std::vector<float> &data) {
          ds.insert(std::end(ds), std::begin(data), std::end(data));
          n++;
        });
      }
      index++;
    }
  }
  if (_record_config.deadlocks) {
    _deadlock_data.resize(_number);
    _record = true;
  }
  if (_record_config.efficacy) {
    _efficacy_data.resize(_run_config.steps * _number);
    _record = true;
  }
}

void ExperimentalRun::start() {
  if (_state == State::init) {
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
  if (_state != State::running or !_record or _steps > _run_config.steps)
    return;
  if (_record_config.time) {
    _time_data.push_back(_world->get_time());
  }
  if (_record_config.pose) {
    size_t i = _steps * _number * 3;
    for (const auto &agent : _world->get_agents()) {
      const auto pose = agent->pose;
      _pose_data[i++] = pose.position[0];
      _pose_data[i++] = pose.position[1];
      _pose_data[i++] = pose.orientation;
    }
  }
  if (_record_config.twist) {
    size_t i = _steps * _number * 3;
    for (const auto &agent : _world->get_agents()) {
      const auto twist = agent->twist;
      _twist_data[i++] = twist.velocity[0];
      _twist_data[i++] = twist.velocity[1];
      _twist_data[i++] = twist.angular_speed;
    }
  }
  if (_record_config.cmd) {
    size_t i = _steps * _number * 3;
    for (const auto &agent : _world->get_agents()) {
      const auto twist = agent->last_cmd;
      _cmd_data[i++] = twist.velocity[0];
      _cmd_data[i++] = twist.velocity[1];
      _cmd_data[i++] = twist.angular_speed;
    }
  }
  if (_record_config.target) {
    size_t i = _steps * _number * 3;
    for (const auto &agent : _world->get_agents()) {
      if (auto b = agent->get_behavior()) {
        // TODO(Jerome): adapt to the changed target format
        const auto target = b->get_target();
        const auto position = target.position.value_or(Vector2::Zero());
        const auto orientation = target.orientation.value_or(0.0f);
        _target_data[i++] = position[0];
        _target_data[i++] = position[1];
        _target_data[i++] = orientation;
      }
    }
  }
  if (_record_config.safety_violation) {
    size_t i = _steps * _number;
    for (const auto &agent : _world->get_agents()) {
      _safety_violation_data[i++] =
          _world->compute_safety_violation(agent.get());
    }
  }
  if (_record_config.collisions) {
    for (const auto &[e1, e2] : _world->get_collisions()) {
      _collisions_data.push_back(_world->get_step());
      _collisions_data.push_back(e1->uid);
      _collisions_data.push_back(e2->uid);
    }
  }
  if (_record_config.efficacy) {
    size_t i = _steps * _number;
    for (const auto &agent : _world->get_agents()) {
      const auto b = agent->get_behavior();
      _efficacy_data[i++] = b ? b->get_efficacy() : 1.0f;
    }
  }
  _steps++;
}

void ExperimentalRun::finalize() {
  if (_record_config.task_events) {
    for (const auto &agent : _world->get_agents()) {
      if (Task *task = agent->get_task()) {
        // TODO(Jerome): should remove just the callback we added.
        task->clear_callbacks();
      }
    }
  }
  if (_record_config.deadlocks) {
    size_t i = 0;
    for (const auto &agent : _world->get_agents()) {
      _deadlock_data[i++] = agent->get_time_since_stuck();
    }
  }
  if (_record && _steps != get_maximal_steps()) {
    if (_record_config.pose) {
      _pose_data.shrink_to_fit();
    }
    if (_record_config.twist) {
      _twist_data.shrink_to_fit();
    }
    if (_record_config.cmd) {
      _cmd_data.shrink_to_fit();
    }
    if (_record_config.target) {
      _target_data.shrink_to_fit();
    }
    if (_record_config.safety_violation) {
      _safety_violation_data.shrink_to_fit();
    }
    if (_record_config.efficacy) {
      _efficacy_data.shrink_to_fit();
    }
  }
}

void ExperimentalRun::save(HighFive::Group &group) {
  store_world(_world_yaml, group);
  store_attribute<float>(_run_config.time_step, "time_step", group);
  store_attribute<unsigned>(_run_config.steps, "maximal_steps", group);
  store_attribute<unsigned>(_steps, "steps", group);
  store_attribute<unsigned>(get_seed(), "seed", group);
  store_attribute<float>(_world->get_time(), "final_sim_time", group);
  const unsigned long d = get_duration_ns().count();
  group
      .createAttribute<unsigned long>("duration_ns",
                                      HighFive::DataSpace::From(d))
      .write(d);
  const std::vector<size_t> dims{static_cast<size_t>(_steps), _number, 3};
  if (_record_config.time) {
    group.createDataSet<float>("times", HighFive::DataSpace({_steps}))
        .write_raw(_time_data.data());
  }
  if (_record_config.pose) {
    group.createDataSet<float>("poses", HighFive::DataSpace(dims))
        .write_raw(_pose_data.data());
  }
  if (_record_config.twist) {
    group.createDataSet<float>("twists", HighFive::DataSpace(dims))
        .write_raw(_twist_data.data());
  }
  if (_record_config.cmd) {
    group.createDataSet<float>("cmds", HighFive::DataSpace(dims))
        .write_raw(_cmd_data.data());
  }
  if (_record_config.target) {
    group.createDataSet<float>("targets", HighFive::DataSpace(dims))
        .write_raw(_target_data.data());
  }
  if (_record_config.safety_violation) {
    group
        .createDataSet<float>("safety_violations",
                              HighFive::DataSpace({_steps, _number}))
        .write_raw(_safety_violation_data.data());
  }
  if (_record_config.collisions) {
    unsigned collisions_number = _collisions_data.size() / 3;
    group
        .createDataSet<unsigned>("collisions",
                                 HighFive::DataSpace({collisions_number, 3}))
        .write_raw(_collisions_data.data());
  }
  if (_record_config.task_events) {
    const auto &agents = _world->get_agents();
    auto task_group = group.createGroup("task_events");
    for (unsigned i = 0; i < _task_events.size(); ++i) {
      const auto n = _task_events[i];
      if (!n) continue;
      const auto ds = _task_events_data[i];
      unsigned uid = agents[i]->uid;
      auto dataset = task_group.createDataSet<float>(
          "agent_" + std::to_string(uid),
          HighFive::DataSpace({n, ds.size() / n}));
      dataset.write_raw(ds.data());
      dataset.createAttribute<unsigned>("uid", HighFive::DataSpace::From(uid))
          .write(uid);
    }
  }
  if (_record_config.deadlocks) {
    group.createDataSet<float>("deadlocks", HighFive::DataSpace({_number}))
        .write_raw(_deadlock_data.data());
  }
  if (_record_config.efficacy) {
    group
        .createDataSet<float>("efficacy",
                              HighFive::DataSpace({_steps, _number}))
        .write_raw(_efficacy_data.data());
  }
}