#include "navground/sim/experiment.h"

#include <filesystem>
#include <fstream>
#include <functional>
#include <highfive/H5DataSpace.hpp>
#include <highfive/H5File.hpp>
#include <string>

#include "navground/core/yaml/yaml.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/experiment.h"

using namespace navground::core;
using namespace navground::sim;

namespace fs = std::filesystem;

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

std::string Experiment::dump() const { return YAML::dump<Experiment>(this); }

void Experiment::init_dataset() {
  file = nullptr;
  if (save_directory.empty()) {
    return;
  }
  fs::current_path(save_directory);
  const std::string yaml = dump();
  const std::size_t hash = std::hash<std::string>{}(yaml);
  std::string dir_name =
      name + "_" + std::to_string(hash) + "_" + file_name_stamp(begin);
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
  store_experiment(yaml, *file);
  store_timepoint(begin, "begin_time", *file);
  store_yaml(yaml);
}

void Experiment::store_yaml(const std::string &yaml) const {
  if (file_path) {
    const fs::path path = file_path->parent_path() / "experiment.yaml";
    std::ofstream out(path);
    if (out.is_open()) {
      out << yaml << std::endl;
    }
  }
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

std::unique_ptr<HighFive::Group> Experiment::init_dataset_run(unsigned index) {
  if (!file || state != State::running) {
    return nullptr;
  }
  auto group = file->createGroup("run_" + std::to_string(index));
  return std::make_unique<HighFive::Group>(std::move(group));
}

ExperimentalRun &Experiment::init_run(int index, std::shared_ptr<World> world) {
  if (!world) {
    world = make_world();
    if (scenario) {
      scenario->init_world(world.get(), index);
    }
  }
  world->prepare();
  runs.try_emplace(index, world, run_config, record_config, index);
  return runs.at(index);
}

void Experiment::run(bool keep, std::optional<unsigned> start_index,
                     std::optional<unsigned> number) {
  start();
  const unsigned max_index =
      start_index.value_or(run_index) + number.value_or(number_of_runs);
  for (size_t i = start_index.value_or(run_index); i < max_index; i++) {
    if (runs.count(i)) continue;
    auto &sim_run = init_run(i);
    sim_run.run();
    save_run(sim_run);
    for (const auto &cb : run_callbacks) {
      cb();
    }
    if (!keep) {
      runs.erase(i);
    }
  }
  stop();
}

void Experiment::save_run(ExperimentalRun &sim_run) {
  auto group = init_dataset_run(sim_run.get_seed());
  if (group) {
    sim_run.save(*group);
  }
}

// TODO()
ExperimentalRun &Experiment::run_once(int index) {
  if (state == State::running) {
    std::cerr << "Should not call run_once when already running an experiment"
              << std::endl;
  }
  remove_run(index);
  auto &sim_run = init_run(index);
  sim_run.run();
  return sim_run;
}

void Experiment::stop_run(ExperimentalRun &sim_run) {
  sim_run.stop();
  save_run(sim_run);
}

void Experiment::start_run(ExperimentalRun &sim_run) {
  sim_run.start();
  start();
}

void Experiment::update_run(ExperimentalRun &sim_run) { sim_run.update(); }

void Experiment::start() {
  if (state == State::running) return;
  begin = std::chrono::system_clock::now();
  experiment_begin = std::chrono::steady_clock::now();
  init_dataset();
  state = State::running;
}

void Experiment::stop() {
  if (state != State::running) return;
  experiment_end = std::chrono::steady_clock::now();
  state = State::finished;
  finalize_dataset();
}
