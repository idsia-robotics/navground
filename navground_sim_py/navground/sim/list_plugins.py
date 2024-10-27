import argparse

from navground import sim
from navground.core.list_plugins import (description, init_parser,
                                         list_plugins, parser)


def _main(arg: argparse.Namespace) -> None:
    sim.load_plugins()
    list_plugins(arg, sim.get_loaded_plugins())


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
