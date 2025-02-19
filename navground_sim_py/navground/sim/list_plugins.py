from __future__ import annotations

import argparse

from navground import sim
from navground.core.list_plugins import description  # noqa: F401
from navground.core.list_plugins import list_plugins, init_parser_with_version


def init_parser(parser: argparse.ArgumentParser) -> None:
    init_parser_with_version(parser, sim.get_build_info().version_string)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def _main(arg: argparse.Namespace) -> None:
    sim.load_plugins()
    list_plugins(arg, sim.get_loaded_plugins(), sim.get_plugins_dependencies())


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
