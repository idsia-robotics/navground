/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "echo.h"
#include "navground/core/command.h"
#include "navground/core/echo.h"
#include "navground/core/info.h"
#include "navground/core/kinematics.h"
#include "navground/core/list_plugins.h"
#include "navground/core/plugins.h"
#include "navground/core/schema.h"
#include "navground/core/yaml/core.h"
#include "navground/sim/build_info.h"
#include "navground/sim/schema.h"
#include "navground/sim/version.h"
#include "navground/sim/yaml/world.h"
#include "run_command.h"
#include "sample_command.h"
#include "yaml-cpp/yaml.h"
#include <iostream>

namespace navground::sim {

namespace core = navground::core;

struct MainCommand : Command<MainCommand> {

  explicit MainCommand(const std::string &name)
      : Command<MainCommand>(name,
                             navground::sim::build_info().get_version_string()),
        _rp("run", navground::sim::build_info().get_version_string()),
        _sp("sample", navground::sim::build_info().get_version_string()),
        _ip("info", navground::sim::build_info().get_version_string()),
        _ep("echo", navground::sim::build_info().get_version_string()),
        _xp("schema", navground::sim::build_info().get_version_string()),
        _pp("plugins", navground::sim::build_info().get_version_string()),
        _run(), _sample(),
        _info("", "",
              {{"Behaviors", navground::core::Behavior::type_properties},
               {"Kinematics", navground::core::Kinematics::type_properties},
               {"Modulations",
                navground::core::BehaviorModulation::type_properties},
               {"State Estimations",
                navground::sim::StateEstimation::type_properties},
               {"Tasks", navground::sim::Task::type_properties},
               {"Scenarios", navground::sim::Scenario::type_properties}},
              navground::sim::get_build_info(),
              navground::sim::get_build_dependencies()),
        _echo("", "", echos()), _schema("", "", "sim", schemas()),
        _list_plugins("", "",
                      {"behaviors", "kinematics", "modulations",
                       "state_estimations", "tasks", "scenarios"}) {}

  void setup(argparse::ArgumentParser &parser) {
    _run.setup(_rp);
    parser.add_subparser(_rp);
    _sample.setup(_sp);
    parser.add_subparser(_sp);
    _info.setup(_ip);
    parser.add_subparser(_ip);
    _echo.setup(_ep);
    sampler_setup(_ep);
    parser.add_subparser(_ep);
    _schema.setup(_xp);
    parser.add_subparser(_xp);
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
    if (parser.is_subcommand_used(_xp)) {
      return _schema.execute(_xp);
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
  argparse::ArgumentParser _xp;
  argparse::ArgumentParser _pp;
  RunCommand _run;
  SampleCommand _sample;
  core::InfoCommand _info;
  core::EchoCommand _echo;
  core::SchemaCommand _schema;
  core::ListPluginsCommand _list_plugins;
};
} // namespace navground::sim

int main(int argc, char *argv[]) {

  return navground::sim::MainCommand("navground").run(argc, argv);
}
