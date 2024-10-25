import argparse
from typing import Any, List, Optional, Tuple, Type

from navground import core
from navground.core import command

Component = Tuple[Type, str, Optional[str]]
Registers = List[Tuple[Type, str]]
registers = [(core.Behavior, "Behaviors"), (core.Kinematics, "Kinematics"),
             (core.BehaviorModulation, "Modulations")]


def get_arg(title: str) -> str:
    return title.lower().replace(' ', '_')


def description() -> str:
    return "Lists registered components."


def add_arg_for_register(parser: argparse.ArgumentParser, title: str) -> None:
    parser.add_argument('--' + get_arg(title),
                        help=f'selects {title.lower()}',
                        type=str,
                        default="",
                        nargs='?',
                        metavar=title.upper())


def init_parser_with_registers(parser: argparse.ArgumentParser,
                               registers: Registers) -> None:
    parser.description = description()
    parser.add_argument('--properties',
                        help="Include properties",
                        action='store_true')
    for _, title in registers:
        add_arg_for_register(parser, title)


def init_parser(parser: argparse.ArgumentParser) -> None:
    command.init_parser(parser)
    init_parser_with_registers(parser, registers)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def print_register(cls: Any,
                   title: Optional[str],
                   name: Optional[str] = None,
                   with_properties: bool = False) -> None:
    if title:
        print(title)
        print("-" * len(title))
    if with_properties:
        for _name, properties in cls.type_properties.items():
            if ((name is not None and _name != name) or len(_name) == 0):
                continue
            print(f"{_name}")
            for k, p in properties.items():
                synonyms = " ".join(p.deprecated_names)
                if synonyms:
                    synonyms = f", deprecated synonyms: {synonyms}"
                print(f"     {k}: {p.default_value} [{p.type_name}]{synonyms}")
    else:
        print(", ".join(t for t in cls.types if t and not (name and t != name)))


def info(arg: argparse.Namespace, registers: Registers) -> None:
    for cls, title in registers:
        name = getattr(arg, get_arg(title))
        selected = name != ''
        if selected:
            print_register(cls, title, name, arg.properties)
            return
    for cls, title in registers:
        print_register(cls, title, None, arg.properties)
        print("")


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    info(arg, registers)


def main() -> None:
    _main(parser().parse_args())
