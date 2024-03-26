import argparse
from typing import Any, Optional, Tuple, Type

from navground import core
# from navground.core import load_plugins

Component = Tuple[Type, str, Optional[str]]

registers = (
    (core.Behavior, "Behaviors", "--behavior"),
    (core.Kinematics, "Kinematics", "--kinematics"))


def add_arg_for_register(parser: argparse.ArgumentParser, arg: str, title: str)-> None:
    parser.add_argument(arg,
                        help=f'selects {title.lower()}',
                        type=str,
                        default="",
                        nargs='?')


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Lists registered components.")
    for _, title, arg in registers:
        add_arg_for_register(parser, arg, title)
    return parser


def print_register(cls: Any, title: Optional[str], name: Optional[str] = None
                   ) -> None:
    if title:
        print(title)
        print("-" * len(title))
    for _name, properties in cls.type_properties.items():
        if (name is not None and _name != name):
            continue
        print(f"{_name}")
        for k, p in properties.items():
            synonyms = " ".join(p.deprecated_names)
            if synonyms:
                synonyms = f", deprecated synonyms: {synonyms}"
            print(f"     {k}: {p.default_value} [{p.type_name}]{synonyms}")


def display_registers(*components: Component) -> None:
    selected = [component[-1] != "" for component in components]
    display = [w or all(not v for j, v in enumerate(selected) if j != i)
               for i, w in enumerate(selected)]
    unique = (len([d for d in display if d]) == 1)
    for d, (cls, title, name) in zip(display, components):
        if d:
            print_register(cls, title if not unique else None, name or None)
            print("")


def main() -> None:
    core.load_plugins()
    arg = parser().parse_args()
    components = [(cls, title, getattr(arg, k.replace('-', '')))
                  for cls, title, k in registers]
    display_registers(*components)
