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
    return 'Samples a world from a scenario or a sampler'


def init_parser(parser: argparse.ArgumentParser) -> None:
    command.init_parser(parser, sim.get_build_info().version_string)
    parser.description = description()
    parser.add_argument(
        'YAML',
        help=
        'YAML string, or path to a YAML file, describing an experiment or a sampler',
        type=str)
    parser.add_argument("--seed", help="The random seed", type=int, default=0)
    parser.add_argument("--type",
                        help="The sampled value",
                        type=str,
                        default="")
    parser.add_argument("--number",
                        help="The number of samples",
                        type=int,
                        default=1)
    parser.add_argument(
        '--chdir',
        help=(
            "Whether to change working directory to the directory containing "
            "the file. Useful when the config contains relative paths."),
        action='store_true')


def execute_scenario(yaml: str, seed: int) -> None:
    try:
        scenario = sim.load_scenario(yaml)
    except RuntimeError as e:
        logging.error(f"Could not load the scenario: {e} {yaml}")
        sys.exit(1)
    if scenario:
        title = "Sampled world"
        print(title)
        print('=' * len(title))
        print('')
        world = scenario.make_world(seed=seed)
        print(sim.dump(world))


def execute_sampler(yaml: str, seed: int, type_name: str, number: int) -> None:
    try:
        sampler = sim.load_sampler(yaml, type_name)
    except RuntimeError as e:
        logging.error(f"Could not load the sampler: {e}")
        sys.exit(1)
    if sampler:
        world = sim.World()
        world.seed = seed
        title = "Sampled " + type_name
        print(title)
        print('=' * len(title))
        print('')
        for i in range(number):
            print(f"{i}: {sampler.sample(world)}")


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
        if arg.type:
            execute_sampler(yaml, arg.seed, arg.type, arg.number)
        else:
            execute_scenario(yaml, arg.seed)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
