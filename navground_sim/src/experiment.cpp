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

static void store_seed(unsigned seed, HighFive::Group &group) {
  group.createAttribute<unsigned>("seed", HighFive::DataSpace::From(seed))
      .write(seed);
}

static void store_steps(unsigned steps, HighFive::Group &group) {
  group.createAttribute<unsigned>("steps", HighFive::DataSpace::From(steps))
      .write(steps);
}

static void store_experiment(const std::string &yaml, HighFive::File &file) {
  file.createAttribute<std::string>("experiment", yaml).write(yaml);
}

static std::string current_stamp() {
  auto time = std::time(nullptr);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time),
                      "%F_%T");  // ISO 8601 without timezone information.
  auto s = ss.str();
  std::replace(s.begin(), s.end(), ':', '-');
  return s;
}

void Trace::init(const World &world, HighFive::Group *group,
                 unsigned max_steps) {
  record = false;
  number = world.get_agents().size();
  const auto & agents = world.get_agents();
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
    for (const auto & agent : world.get_agents()) {
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

void Trace::finalize(const World &world, HighFive::Group *group) {
  if (!group) return;
  const std::vector<size_t> dims{static_cast<size_t>(steps), number, 3};
  if (record_pose) {
    group->createDataSet<float>("pose", HighFive::DataSpace(dims))
        .write_raw(pose_data.data());
  }
  if (record_twist) {
    group->createDataSet<float>("twist", HighFive::DataSpace(dims))
        .write_raw(twist_data.data());
  }
  if (record_cmd) {
    group->createDataSet<float>("cmd", HighFive::DataSpace(dims))
        .write_raw(cmd_data.data());
  }
  if (record_target) {
    group->createDataSet<float>("target", HighFive::DataSpace(dims))
        .write_raw(target_data.data());
  }
  if (record_safety_violation) {
    group
        ->createDataSet<float>("safety_violation",
                               HighFive::DataSpace({steps, number}))
        .write_raw(safety_violation_data.data());
  }
  if (record_collisions) {
    unsigned collisions_number = collisions_data.size() / 3;
    group
        ->createDataSet<unsigned>("collisions",
                                  HighFive::DataSpace({collisions_number, 3}))
        .write_raw(collisions_data.data());
  }
  if (record_task_events) {
    for (unsigned i = 0; i < task_events.size(); ++i) {
      const auto n = task_events[i];
      if (!n) continue;
      const auto ds = task_events_data[i];
      group->createGroup("agent_" + std::to_string(i))
          .createDataSet<float>("task", HighFive::DataSpace({n, ds.size() / n}))
          .write_raw(ds.data());
    }
    for (const auto & agent : world.get_agents()) {
      if (Task *task = agent->get_task()) {
        // TODO(Jerome): should remove just the callback we added.
        task->clear_callbacks();
      }
    }
  }
}

void Experiment::init_run(int index) {
  set_random_seed(index);
  world = make_world();
  if (scenario) {
    scenario->init_world(world.get());
  }
}

void Experiment::init_dataset() {
  file = nullptr;
  if (save_directory.empty()) {
    return;
  }
  fs::current_path(save_directory);
  const std::string dir_name = name + "_" + current_stamp();
  const fs::path dir = save_directory / dir_name;
  if (!create_directory(dir)) {
    std::cerr << "Could not create directory " << dir << std::endl;
    return;
  }
  fs::current_path(dir);
  file = std::make_shared<HighFive::File>("data.h5", HighFive::File::Truncate);
  store_experiment(dump(), *file);
}

std::string Experiment::dump() { return YAML::dump<Experiment>(this); }

void Experiment::init_dataset_run(unsigned index) {
  if (!file) {
    run_group = nullptr;
    return;
  }
  auto group = file->createGroup("run_" + std::to_string(index));
  store_world(*world, group);
  store_seed(get_random_seed(), group);
  run_group = std::make_shared<HighFive::Group>(std::move(group));
}

void Experiment::run() {
  for (size_t i = 0; i < runs; i++) {
    run_once(i);
  }
}

void Experiment::run_once(int index) {
  if (!initialized) {
    init_dataset();
    initialized = true;
  }
  init_run(index);
  if (!world) {
    std::cerr << "No world" << std::endl;
    return;
  }
  world->prepare();
  init_dataset_run(index);
  trace.init(*world, run_group.get(), steps);
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
  trace.finalize(*world, run_group.get());
  if (run_group) {
    store_steps(steps, *run_group);
  }
}
