import argparse

from navground import sim


def main() -> None:
    # nav.load_plugins()
    sim.load_py_plugins()
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml', help='yaml string', type=str, default="{}")
    arg = parser.parse_args()
    scenario = sim.load_scenario(arg.yaml)
    if scenario:
        print(sim.dump(scenario))
        print("==========\n")
        world = sim.World()
        scenario.init_world(world)
        print(sim.dump(world))
    else:
        print("could not load yaml")
