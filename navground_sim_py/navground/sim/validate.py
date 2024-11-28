import argparse

from navground import sim
from navground.core import command, validate

kinds: list[str] = validate.kinds + [
    "state_estimation", "task", "agent", "world", "scenario", "experiment"
]

description = validate.description


def init_parser(parser: argparse.ArgumentParser) -> None:
    validate.init_parser_with_kinds(parser, kinds)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def _main(arg: argparse.Namespace) -> None:
    command._main(arg, sim.load_plugins)
    validate.validate(arg, kinds, sim.schema.bundle())


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
