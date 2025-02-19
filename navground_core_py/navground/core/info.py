from __future__ import annotations

import argparse
from typing import Any

from navground import core
from navground.core import command

Component = tuple[type[Any], str, str | None]
Registers = list[tuple[type[Any], str]]
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


def init_parser_with_registers(parser: argparse.ArgumentParser, version: str,
                               registers: Registers) -> None:
    command.init_parser(parser, version)
    parser.description = description()
    parser.add_argument('--build',
                        help="Include build infos",
                        action='store_true')
    parser.add_argument('--properties',
                        help="Include properties",
                        action='store_true')
    parser.add_argument('--description',
                        help="Include property descriptions",
                        action='store_true')
    for _, title in registers:
        add_arg_for_register(parser, title)


def init_parser(parser: argparse.ArgumentParser) -> None:
    init_parser_with_registers(parser,
                               core.get_build_info().version_string, registers)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def print_register(cls: Any,
                   title: str | None,
                   name: str | None = None,
                   with_properties: bool = False,
                   with_description: bool = False) -> None:
    if title:
        print(title)
        print("-" * len(title))
    if with_properties:
        for _name, properties in cls.type_properties.items():
            if ((name is not None and _name != name) or len(_name) == 0):
                continue
            print(f"{_name}")
            for k, p in properties.items():
                readonly = " readonly" if p.readonly else ""
                synonyms = " ".join(p.deprecated_names)
                if synonyms:
                    synonyms = f", deprecated synonyms: {synonyms}"
                print(
                    f"    {k}: {p.default_value} ({p.type_name}){readonly}{synonyms}"
                )
                if with_description and p.description:
                    print(f"      {p.description}")
    else:
        print(", ".join(t for t in cls.types
                        if t and not (name and t != name)))


def info(arg: argparse.Namespace, registers: Registers,
         build_info: core.BuildInfo,
         build_dependencies: core.BuildDependencies) -> None:
    if arg.build:
        print(f"Build: {build_info}")
        print("Dependencies:")
        for name, dep in build_dependencies.items():
            print(f"- {name}: {dep}")
        print("")
    print("Installed components")
    print("====================")
    for cls, title in registers:
        name = getattr(arg, get_arg(title))
        selected = name != ''
        if selected:
            print_register(cls, title, name, arg.properties, arg.description)
            return
    for cls, title in registers:
        print_register(cls, title, None, arg.properties, arg.description)
        print("")


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    info(arg, registers, core.get_build_info(), core.get_build_dependencies())


def main() -> None:
    _main(parser().parse_args())
