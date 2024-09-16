import argparse

from navground import sim
from navground.core.info import registers as core_registers
from navground.core.info import display_registers, add_arg_for_register

registers = list(core_registers) + [
    (sim.StateEstimation, "State estimations", "--state_estimation"),
    (sim.Task, "Tasks", "--task"), (sim.Scenario, "Scenarios", "--scenario")
]


def init_parser(parser: argparse.ArgumentParser) -> None:
    parser.description = "Lists registered components"
    for _, title, arg in registers:
        add_arg_for_register(parser, arg, title)


def _main(arg: argparse.Namespace) -> None:
    sim.load_plugins()
    components = [(cls, title, getattr(arg, k.replace('-', '')))
                  for cls, title, k in registers]
    display_registers(*components)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
