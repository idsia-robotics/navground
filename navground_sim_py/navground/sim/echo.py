import argparse

from navground import sim
from navground.core import echo

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
    echo.init_parser(parser, echos)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def _main(arg: argparse.Namespace) -> None:
    echo.echo(arg, echos, sim.dump)


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
