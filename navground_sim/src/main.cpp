/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/command.h"
#include "navground/core/info.h"
#include "navground/core/plugins.h"
#include "run_command.h"
#include "sample_command.h"
#include "yaml-cpp/yaml.h"

namespace navground::sim {

struct MainCommand : Command<MainCommand> {

  explicit MainCommand(const std::string &name)
      : Command<MainCommand>(name), _rp("run"), _sp("sample"), _ip("info"),
        _run(), _sample(),
        _info("", {{"Behaviors", navground::core::Behavior::type_properties},
                   {"Kinematics", navground::core::Kinematics::type_properties},
                   {"Modulations",
                    navground::core::BehaviorModulation::type_properties},
                   {"State Estimations",
                    navground::sim::StateEstimation::type_properties},
                   {"Tasks", navground::sim::Task::type_properties},
                   {"Scenarios", navground::sim::Scenario::type_properties}}) {}

  void setup(argparse::ArgumentParser &parser) {
    _run.setup(_rp);
    parser.add_subparser(_rp);
    _sample.setup(_sp);
    parser.add_subparser(_sp);
    _info.setup(_ip);
    parser.add_subparser(_ip);
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
    return 1;
  }

private:
  argparse::ArgumentParser _rp;
  argparse::ArgumentParser _sp;
  argparse::ArgumentParser _ip;
  RunCommand _run;
  SampleCommand _sample;
  core::InfoCommand _info;
};
} // namespace navground::sim

int main(int argc, char *argv[]) {

  return navground::sim::MainCommand("navground").run(argc, argv);
}
