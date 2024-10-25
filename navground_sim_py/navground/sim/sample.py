import argparse
import logging
import os
import sys

from navground import sim
from navground.core import command


def description() -> str:
    return 'Samples a world from a scenario.'


def init_parser(parser: argparse.ArgumentParser) -> None:
    command.init_parser(parser)
    parser.description = description()
    parser.add_argument(
        'YAML',
        help='YAML string, or path to a YAML file, describing an experiment',
        type=str)


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    logging.basicConfig(level=logging.INFO)
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


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
