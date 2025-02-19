from __future__ import annotations

import argparse

from navground import sim
from navground.core import command, print_schema

schemas: print_schema.Schemas = print_schema.schemas | {
    "sim": sim.schema.bundle,
    "agent": sim.Agent.schema,
    "world": sim.World.schema,
    "experiment": sim.World.schema,
}

components: print_schema.Components = print_schema.components | {
    "state_estimation": sim.StateEstimation,
    "task": sim.Task,
    "scenario": sim.Scenario,
}

description = print_schema.description


def init_parser(parser: argparse.ArgumentParser) -> None:
    print_schema.init_parser_with_schemas(parser,
                                          sim.get_build_info().version_string,
                                          'sim', schemas, components)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def _main(arg: argparse.Namespace) -> None:
    command._main(arg, sim.load_plugins)
    print_schema.schema(arg, schemas, components)


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
