/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <chrono>
#include <memory>

#include "navground/sim/experiment.h"
#include "navground/sim/world.h"
#include "scenario.h"

using navground::sim::Experiment;
using navground::sim::World;

static void show_usage(std::string name) {
  std::cout << "Usage: " << name << " <option(s)>" << std::endl
            << "Options:" << std::endl
            << "  --help\t\t\tShow this help message" << std::endl
            << "  --behavior=<BEHAVIOR>\tNavigation behavior" << std::endl;
}

int main(int argc, char* argv[]) {
  char behavior_name[10] = "HL";
  for (int i = 0; i < argc; i++) {
    if (sscanf(argv[i], "--behavior=%9s", behavior_name)) {
      continue;
    } else if (sscanf(argv[i], "--help")) {
      show_usage(argv[0]);
      return 0;
    }
  }
  Experiment demo(0.02, 50 * 60);
  demo.trace.record_pose = true;
  demo.save_directory = ".";
  demo.scenario = std::make_shared<ThymioDemo>(behavior_name);
  demo.name = "ThymioDemo";
  printf("Start simulating 1 minute at 50 ticks per second\n");
  auto begin = std::chrono::high_resolution_clock::now();
  demo.run();
  auto end = std::chrono::high_resolution_clock::now();
  unsigned us =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
  printf("Done simulating in %.1f ms\n", us * 1e-6);
  return 0;
}
