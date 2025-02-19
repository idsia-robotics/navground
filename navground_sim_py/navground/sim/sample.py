from __future__ import annotations

import argparse
import logging
import os
import pathlib
import sys

from navground import sim
from navground.core import command
from navground.core.utils import chdir


def description() -> str:
    return 'Samples a world from a scenario.'


def init_parser(parser: argparse.ArgumentParser) -> None:
    command.init_parser(parser, sim.get_build_info().version_string)
    parser.description = description()
    parser.add_argument(
        'YAML',
        help='YAML string, or path to a YAML file, describing an experiment',
        type=str)
    parser.add_argument(
        '--chdir',
        help=(
            "Whether to change working directory to the directory containing "
            "the file. Useful when the config contains relative paths."),
        action='store_true')


def _main(arg: argparse.Namespace) -> None:
    command._main(arg, sim.load_plugins)
    logging.basicConfig(level=logging.INFO)
    wd: pathlib.Path | None = None
    if os.path.exists(arg.YAML) and os.path.isfile(arg.YAML):
        with open(arg.YAML) as f:
            yaml = f.read()
        if arg.chdir:
            wd = pathlib.Path(arg.YAML).parent
    else:
        yaml = arg.YAML
    with chdir(wd):
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
