from typing import Any

from navground import core
from navground import sim


def print_register(cls: Any, title: str) -> None:
    print(title)
    print("=" * len(title))
    for name, properties in cls.type_properties.items():
        print(f"- {name}")
        for k, p in properties.items():
            print(f"     {k}: {p.default_value} [{p.type_name}]")


def main() -> None:
    # nav.load_plugins()
    sim.load_py_plugins()
    print_register(core.Behavior, "Behaviors")
    print("")
    print_register(core.Kinematics, "Kinematics")
    print("")
    print_register(sim.StateEstimation, "State estimations")
    print("")
    print_register(sim.Task, "Tasks")
    print("")
    print_register(sim.Scenario, "Scenarios")
