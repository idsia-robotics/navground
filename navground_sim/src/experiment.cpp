#include "navground/sim/experiment.h"

#include <filesystem>
#include <highfive/H5DataSpace.hpp>
#include <highfive/H5File.hpp>

#include "navground/core/yaml/yaml.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/experiment.h"
#include "navground/sim/yaml/world.h"

using namespace navground::core;
using namespace navground::sim;

namespace fs = std::filesystem;

static void store_world(const World &world, HighFive::Group &group) {
  const std::string yaml = YAML::dump<World>(&world);
  group.createAttribute<std::string>("world", HighFive::DataSpace::From(yaml))
      .write(yaml);
}

template <typename T>
static void store_attribute(T value, const std::string &name,
                            HighFive::Group &group) {
  group.createAttribute<T>(name, HighFive::DataSpace::From(value)).write(value);
}

static void store_experiment(const std::string &yaml, HighFive::File &file) {
  file.createAttribute<std::string>("experiment", yaml).write(yaml);
}

static std::string time_string(
    const std::chrono::time_point<std::chrono::system_clock> &t) {
  const auto ts = std::chrono::system_clock::to_time_t(t);
  std::stringstream ss;
  // ISO 8601 without timezone information.
  ss << std::put_time(std::localtime(&ts), "%F_%T");
  return ss.str();
}

static void store_timepoint(
    const std::chrono::time_point<std::chrono::system_clock> &tp,
    const std::string &name, HighFive::File &file) {
  const std::string txt = time_string(tp);
  file.createAttribute<std::string>(name, txt).write(txt);
}

static std::string file_name_stamp(
    const std::chrono::time_point<std::chrono::system_clock> &t) {
  auto s = time_string(t);
  std::replace(s.begin(), s.end(), ':', '-');
  return s;
}

void Trace::init(const World &world, unsigned max_steps) {
  record = false;
  number = world.get_agents().size();
  const auto &agents = world.get_agents();
  indices.clear();
  for (unsigned i = 0; i < number; ++i) {
    indices[agents[i].get()] = i;
  }
  steps = 0;
  if (record_pose) {
    pose_data.clear();
    pose_data.reserve(max_steps * number * 3);
    pose_ptr = reinterpret_cast<float *>(pose_data.data());
    record = true;
  }
  if (record_twist) {
    twist_data.clear();
    twist_data.reserve(max_steps * number * 3);
    twist_ptr = reinterpret_cast<float *>(twist_data.data());
    record = true;
  }
  if (record_cmd) {
    cmd_data.clear();
    cmd_data.reserve(max_steps * number * 3);
    cmd_ptr = reinterpret_cast<float *>(cmd_data.data());
    record = true;
  }
  if (record_target) {
    target_data.clear();
    target_data.reserve(max_steps * number * 3);
    target_ptr = reinterpret_cast<float *>(target_data.data());
    record = true;
  }
  if (record_safety_violation) {
    safety_violation_data.clear();
    safety_violation_data.reserve(max_steps * number);
    safety_violation_ptr =
        reinterpret_cast<float *>(safety_violation_data.data());
    record = true;
  }
  if (record_collisions) {
    record = true;
    collisions_data.clear();
  }
  if (record_task_events) {
    record = true;
    task_events_data = std::vector<std::vector<float>>(number);
    task_events = std::vector<unsigned>(number, 0);
    int index = 0;
    for (const auto &agent : world.get_agents()) {
      if (Task *task = agent->get_task()) {
        auto &ds = task_events_data[index];
        auto &n = task_events[index];
        task->add_callback([&ds, &n](const std::vector<float> &data) {
          ds.insert(std::end(ds), std::begin(data), std::end(data));
          n++;
        });
      }
      index++;
    }
  }
}

void Trace::update(const World &world, unsigned step) {
  if (!record) return;
  steps++;
  if (record_pose) {
    for (const auto &agent : world.get_agents()) {
      const auto pose = agent->pose;
      *pose_ptr++ = pose.position[0];
      *pose_ptr++ = pose.position[1];
      *pose_ptr++ = pose.orientation;
    }
  }
  if (record_twist) {
    for (const auto &agent : world.get_agents()) {
      const auto twist = agent->twist;
      *twist_ptr++ = twist.velocity[0];
      *twist_ptr++ = twist.velocity[1];
      *twist_ptr++ = twist.angular_speed;
    }
  }
  if (record_cmd) {
    for (const auto &agent : world.get_agents()) {
      const auto twist = agent->last_cmd;
      *cmd_ptr++ = twist.velocity[0];
      *cmd_ptr++ = twist.velocity[1];
      *cmd_ptr++ = twist.angular_speed;
    }
  }
  if (record_target) {
    for (const auto &agent : world.get_agents()) {
      if (auto b = agent->get_behavior()) {
        // TODO(Jerome): adapt to the changed target format
        const auto target = b->get_target();
        const auto position = target.position.value_or(Vector2::Zero());
        const auto orientation = target.orientation.value_or(0.0f);
        *target_ptr++ = position[0];
        *target_ptr++ = position[1];
        *target_ptr++ = orientation;
      }
    }
  }
  if (record_safety_violation) {
    for (const auto &agent : world.get_agents()) {
      *safety_violation_ptr++ = world.compute_safety_violation(agent.get());
    }
  }
  if (record_collisions) {
    const auto collisions = world.get_collisions();
    const unsigned n = collisions.size();
    if (n > 0) {
      collisions_data.reserve(collisions_data.size() + 3 * n);
      for (const auto &[e1, e2] : collisions) {
        collisions_data.push_back(step);
        collisions_data.push_back(e1->uid);
        collisions_data.push_back(e2->uid);
      }
    }
  }
}

void Trace::finalize(const World &world) {
  if (record_task_events) {
    for (const auto &agent : world.get_agents()) {
      if (Task *task = agent->get_task()) {
        // TODO(Jerome): should remove just the callback we added.
        task->clear_callbacks();
      }
    }
  }
}

void Trace::save(const World &world, HighFive::Group &group) {
  const std::vector<size_t> dims{static_cast<size_t>(steps), number, 3};
  if (record_pose) {
    group.createDataSet<float>("poses", HighFive::DataSpace(dims))
        .write_raw(pose_data.data());
  }
  if (record_twist) {
    group.createDataSet<float>("twists", HighFive::DataSpace(dims))
        .write_raw(twist_data.data());
  }
  if (record_cmd) {
    group.createDataSet<float>("cmds", HighFive::DataSpace(dims))
        .write_raw(cmd_data.data());
  }
  if (record_target) {
    group.createDataSet<float>("targets", HighFive::DataSpace(dims))
        .write_raw(target_data.data());
  }
  if (record_safety_violation) {
    group
        .createDataSet<float>("safety_violations",
                              HighFive::DataSpace({steps, number}))
        .write_raw(safety_violation_data.data());
  }
  if (record_collisions) {
    unsigned collisions_number = collisions_data.size() / 3;
    group
        .createDataSet<unsigned>("collisions",
                                 HighFive::DataSpace({collisions_number, 3}))
        .write_raw(collisions_data.data());
  }
  if (record_task_events) {
    const auto &agents = world.get_agents();
    auto task_group = group.createGroup("task_events");
    for (unsigned i = 0; i < task_events.size(); ++i) {
      const auto n = task_events[i];
      if (!n) continue;
      const auto ds = task_events_data[i];
      unsigned uid = agents[i]->uid;
      auto dataset = task_group.createDataSet<float>(
          "agent_" + std::to_string(uid),
          HighFive::DataSpace({n, ds.size() / n}));
      dataset.write_raw(ds.data());
      dataset.createAttribute<unsigned>("uid", HighFive::DataSpace::From(uid))
          .write(uid);
    }
  }
}

void Experiment::init_run(int index) {
  set_random_seed(index);
  world = make_world();
  if (scenario) {
    scenario->init_world(world.get());
  }
  world->prepare();
}

void Experiment::init_dataset() {
  file = nullptr;
  if (save_directory.empty()) {
    return;
  }
  fs::current_path(save_directory);
  std::string dir_name = name + "_" + file_name_stamp(begin);
  if (std::filesystem::exists(save_directory / dir_name)) {
    int i = 0;
    for (;; i++) {
      if (!std::filesystem::exists(save_directory /
                                   (dir_name + std::to_string(i)))) {
        break;
      }
    }
    dir_name += "_" + std::to_string(i);
    std::cout << "Added suffix _" + std::to_string(i) << std::endl;
  }
  const fs::path dir = save_directory / dir_name;
  if (!create_directory(dir)) {
    std::cerr << "Could not create directory " << dir << std::endl;
    return;
  }
  file_path = dir / "data.h5";
  file = std::make_shared<HighFive::File>(*file_path, HighFive::File::Truncate);
  store_experiment(dump(), *file);
  store_timepoint(begin, "begin_time", *file);
}

void Experiment::finalize_dataset() {
  if (file) {
    unsigned long d = get_duration_ns().count();
    file->createAttribute<unsigned long>("duration_ns",
                                         HighFive::DataSpace::From(d))
        .write(d);
  }
  file = nullptr;
}

std::string Experiment::dump() { return YAML::dump<Experiment>(this); }

void Experiment::init_dataset_run(unsigned index) {
  if (!file) {
    run_group = nullptr;
    return;
  }
  auto group = file->createGroup("run_" + std::to_string(index));
  if (world) {
    store_world(*world, group);
  }
  store_attribute<unsigned>(get_random_seed(), "seed", group);
  store_attribute<float>(time_step, "time_step", group);
  run_group = std::make_shared<HighFive::Group>(std::move(group));
}

void Experiment::finalize_dataset_run() {
  if (run_group) {
    if (world) {
      trace.save(*world, *run_group);
    }
    store_attribute<unsigned>(steps, "steps", *run_group);
    unsigned long d = get_run_duration_ns().count();
    run_group
        ->createAttribute<unsigned long>("duration_ns",
                                         HighFive::DataSpace::From(d))
        .write(d);
  }
  run_group = nullptr;
}

void Experiment::run() {
  begin = std::chrono::system_clock::now();
  experiment_begin = std::chrono::steady_clock::now();
  init_dataset();
  for (size_t i = 0; i < runs; i++) {
    init_run(i);
    init_dataset_run(i);
    run_run();
    finalize_dataset_run();
  }
  experiment_end = std::chrono::steady_clock::now();
  finalize_dataset();
}

void Experiment::run_once(int index) {
  init_run(index);
  run_run();
}

void Experiment::run_run() {
  if (!world) return;
  trace.init(*world, steps);
  run_begin = std::chrono::steady_clock::now();
  for (step = 0; step < steps; step++) {
    world->update(time_step);
    trace.update(*world, step);
    for (const auto &cb : callbacks) {
      cb();
    }
    if (terminate_when_all_idle && world->agents_are_idle()) {
      break;
    }
  }
  run_end = std::chrono::steady_clock::now();
  trace.finalize(*world);
}

void Experiment::start(int index) {
  begin = std::chrono::system_clock::now();
  experiment_begin = std::chrono::steady_clock::now();
  set_random_seed(index);
  world->prepare();
  init_dataset();
  init_dataset_run(index);
  trace.init(*world, steps);
  run_begin = std::chrono::steady_clock::now();
}

void Experiment::stop() {
  run_end = std::chrono::steady_clock::now();
  trace.finalize(*world);
  finalize_dataset_run();
  experiment_end = std::chrono::steady_clock::now();
  finalize_dataset();
}

void Experiment::update() {
  trace.update(*world, world->get_step() - 1);
}
