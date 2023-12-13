import argparse
import logging
import os
import sys

from navground import sim


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Samples a world from a scenario.')
    parser.add_argument('YAML',
                        help='YAML string, or path to a YAML file, describing an experiment',
                        type=str)
    return parser


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    # nav.load_plugins()
    sim.load_py_plugins()
    arg = parser().parse_args()
    if os.path.exists(arg.YAML) and os.path.isfile(arg.YAML):
        with open(arg.YAML, 'r') as f:
            yaml = f.read()
    else:
        yaml = arg.YAML
    try:
        scenario = sim.load_scenario(yaml)
    except RuntimeError as e:
        logging.error(f"Could not load the scenario: {e} {arg.YAML}")
        sys.exit(1)
    if scenario:
        print(sim.dump(scenario))
        print('-' * 30)
        world = sim.World()
        scenario.init_world(world)
        print(sim.dump(world))
