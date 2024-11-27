import argparse
from types import ModuleType
from typing import Any

from . import echo, info, list_plugins, print_schema, validate


def config_parser(parsers: Any, name: str, module: ModuleType) -> None:
    parser = parsers.add_parser(name, help=module.description())
    module.init_parser(parser)
    parser.set_defaults(func=module._main)


def init_parser(parser: argparse.ArgumentParser) -> None:
    parsers = parser.add_subparsers(dest='cmd', title='Subcommands')
    config_parser(parsers, 'info', info)
    config_parser(parsers, 'echo', echo)
    config_parser(parsers, 'plugins', list_plugins)
    config_parser(parsers, 'schema', print_schema)
    config_parser(parsers, 'validate', validate)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='navground_py')
    init_parser(parser)
    return parser


def main() -> None:
    ps = parser()
    args = ps.parse_args()
    if args.cmd is not None:
        args.func(args)
    else:
        print("Welcome to navground!\n")
        ps.print_help()
