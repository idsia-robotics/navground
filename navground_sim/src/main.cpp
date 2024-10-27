/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/command.h"
#include "navground/core/echo.h"
#include "navground/core/info.h"
#include "navground/core/kinematics.h"
#include "navground/core/list_plugins.h"
#include "navground/core/plugins.h"
#include "navground/core/yaml/core.h"
#include "navground/sim/yaml/experiment.h"
#include "navground/sim/yaml/scenario.h"
#include "navground/sim/yaml/world.h"
#include "run_command.h"
#include "sample_command.h"
#include "yaml-cpp/yaml.h"
#include <iostream>

namespace navground::sim {

struct MainCommand : Command<MainCommand> {

  explicit MainCommand(const std::string &name)
      : Command<MainCommand>(name), _rp("run"), _sp("sample"), _ip("info"),
        _ep("echo"), _pp("plugins"), _run(), _sample(),
        _info("", {{"Behaviors", navground::core::Behavior::type_properties},
                   {"Kinematics", navground::core::Kinematics::type_properties},
                   {"Modulations",
                    navground::core::BehaviorModulation::type_properties},
                   {"State Estimations",
                    navground::sim::StateEstimation::type_properties},
                   {"Tasks", navground::sim::Task::type_properties},
                   {"Scenarios", navground::sim::Scenario::type_properties}}),
        _echo("",
              {
                  {"behavior", &core::echo<core::Behavior>},
                  {"modulation", &core::echo<core::BehaviorModulation>},
                  {"kinematics", &core::echo<core::Kinematics>},
                  {"state_estimation", &core::echo<sim::StateEstimation>},
                  {"task", &core::echo<sim::Task>},
                  {"scenario", &core::echo<sim::Scenario>},
                  {"world", &core::echo_s<sim::World>},
                  {"agent", &core::echo_s<sim::Agent>},
                  {"experiment", &core::echo_s<sim::Experiment>},
              }),
        _list_plugins("", {"behaviors", "kinematics", "modulations",
                           "state_estimations", "tasks", "scenarios"}) {}

  void setup(argparse::ArgumentParser &parser) {
    _run.setup(_rp);
    parser.add_subparser(_rp);
    _sample.setup(_sp);
    parser.add_subparser(_sp);
    _info.setup(_ip);
    parser.add_subparser(_ip);
    _echo.setup(_ep);
    parser.add_subparser(_ep);
    _list_plugins.setup(_pp);
    parser.add_subparser(_pp);
  }

  int execute(const argparse::ArgumentParser &parser) {
    if (parser.is_subcommand_used(_rp)) {
      return _run.execute(_rp);
    }
    if (parser.is_subcommand_used(_sp)) {
      return _sample.execute(_sp);
    }
    if (parser.is_subcommand_used(_ip)) {
      return _info.execute(_ip);
    }
    if (parser.is_subcommand_used(_ep)) {
      return _echo.execute(_ep);
    }
    if (parser.is_subcommand_used(_pp)) {
      return _list_plugins.execute(_pp);
    }
    std::cout << "Welcome to navground!" << std::endl << std::endl;
    std::cout << parser << std::endl;
    return 0;
  }

private:
  argparse::ArgumentParser _rp;
  argparse::ArgumentParser _sp;
  argparse::ArgumentParser _ip;
  argparse::ArgumentParser _ep;
  argparse::ArgumentParser _pp;
  RunCommand _run;
  SampleCommand _sample;
  core::InfoCommand _info;
  core::EchoCommand _echo;
  core::ListPluginsCommand _list_plugins;
};
} // namespace navground::sim

int main(int argc, char *argv[]) {

  return navground::sim::MainCommand("navground").run(argc, argv);
}
