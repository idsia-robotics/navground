import argparse

from navground import sim
from navground.core import command, print_schema

schemas: print_schema.Schemas = print_schema.schemas | {
    "sim": sim.schema,
    "state_estimation": sim.StateEstimation.base_schema,
    "task": sim.Task.base_schema,
    "agent": sim.Agent.schema,
    "world": sim.World.schema,
    "scenario": sim.Scenario.base_schema,
    "experiment": sim.World.schema,
    "state_estimation_register": sim.StateEstimation.register_schema,
    "task_register": sim.Task.register_schema,
    "scenario_register": sim.Scenario.register_schema,
}

description = print_schema.description


def init_parser(parser: argparse.ArgumentParser) -> None:
    print_schema.init_parser_with_schemas(parser, 'sim', schemas)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def _main(arg: argparse.Namespace) -> None:
    command._main(arg, sim.load_plugins)
    print_schema.schema(arg, schemas)


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
