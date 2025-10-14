from __future__ import annotations

import argparse
import logging
import sys
from typing import Any

from navground import sim
from navground.core import command, echo

echos: echo.Echos = echo.echos | {
    "state_estimation": sim.load_state_estimation,
    "task": sim.load_task,
    "scenario": sim.load_scenario,
    "world": sim.load_world,
    "agent": sim.load_agent,
    "experiment": sim.load_experiment,
}

description = echo.description


def init_parser(parser: argparse.ArgumentParser) -> None:
    echo.init_parser_with_kinds(parser,
                                sim.get_build_info().version_string,
                                list(echos.keys()) + ["sampler"])
    parser.add_argument("--type",
                        type=str,
                        help="The sampled type",
                        default='',
                        choices=[
                            'bool', 'int', 'float', 'str', 'vector', '[bool]',
                            '[int]', '[float]', '[str]', '[vector]'
                        ])


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def load(arg: argparse.Namespace, yaml: str) -> Any:
    if arg.kind == 'sampler':
        return sim.load_sampler(yaml, arg.type)
    if arg.kind not in echos:
        logging.error(f"Unknown kind of object to load: {arg.kind}")
        sys.exit(1)
    return echos[arg.kind](yaml)


def _main(arg: argparse.Namespace) -> None:
    command._main(arg, sim.load_plugins)
    echo.echo(arg, load, sim.dump)


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
