from typing import Any

from navground.core import Behavior, Kinematics
# from navground.core import load_plugins


def print_register(cls: Any, title: str) -> None:
    print(title)
    print("=" * len(title))
    print("")
    for name, properties in cls.type_properties.items():
        print(f"- {name}")
        for k, p in properties.items():
            print(f"     {k}: {p.default_value} [{p.type_name}]")


def main() -> None:
    # load_plugins()
    print_register(Behavior, "Behaviors")
    print("")
    print_register(Kinematics, "Kinematics")
