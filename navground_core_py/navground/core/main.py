import argparse

from . import info
from . import echo
from . import list_plugins
from . import validate
from . import print_schema


def config_parser(parsers, name, module):
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


def main():
    ps = parser()
    args = ps.parse_args()
    if args.cmd is not None:
        args.func(args)
    else:
        print("Welcome to navground!\n")
        ps.print_help()
