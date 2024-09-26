import argparse

from navground import sim
from navground.core.info import description, info, init_parser_with_registers
from navground.core.info import registers as core_registers

registers = core_registers + [(sim.StateEstimation, "State Estimations"),
                              (sim.Task, "Tasks"), (sim.Scenario, "Scenarios")]


def init_parser(parser: argparse.ArgumentParser) -> None:
    init_parser_with_registers(parser, registers)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


def _main(arg: argparse.Namespace) -> None:
    info(arg, registers)


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
