import argparse
from types import ModuleType
from typing import Any

from . import (echo, info, list_plugins, print_schema, record_video, replay,
               run, run_rt, sample, validate)


def config_parser(parsers: Any, name: str, module: ModuleType) -> None:
    parser = parsers.add_parser(name, help=module.description())
    module.init_parser(parser)
    parser.set_defaults(func=module._main)


def init_parser(parser: argparse.ArgumentParser) -> None:
    parsers = parser.add_subparsers(dest='cmd',
                                    title='Subcommands',
                                    metavar="{info,run,...}")
    config_parser(parsers, 'info', info)
    config_parser(parsers, 'echo', echo)
    config_parser(parsers, 'schema', print_schema)
    config_parser(parsers, 'validate', validate)
    config_parser(parsers, 'plugins', list_plugins)
    config_parser(parsers, 'run', run)
    config_parser(parsers, 'run_rt', run_rt)
    config_parser(parsers, 'sample', sample)
    config_parser(parsers, 'record_video', record_video)
    config_parser(parsers, 'replay', replay)


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='navground_py')
    init_parser(parser)
    return parser


def _main(arg: argparse.Namespace) -> None:
    if arg.cmd is not None:
        arg.func(arg)
    else:
        print("Welcome to navground!\n")
        parser().print_help()


def main() -> None:
    args = parser().parse_args()
    _main(args)
