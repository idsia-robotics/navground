import argparse

from navground import sim
from navground.core.list import registers as core_registers
from navground.core.list import display_registers, add_arg_for_register

registers = list(core_registers) + [
    (sim.StateEstimation, "State estimations", "--state_estimation"),
    (sim.Task, "Tasks", "--task"), (sim.Scenario, "Scenarios", "--scenario")
]


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Lists registered components.")
    for _, title, arg in registers:
        add_arg_for_register(parser, arg, title)
    return parser


def main() -> None:
    sim.load_py_plugins()
    arg = parser().parse_args()
    components = [(cls, title, getattr(arg, k.replace('-', '')))
                  for cls, title, k in registers]
    display_registers(*components)
