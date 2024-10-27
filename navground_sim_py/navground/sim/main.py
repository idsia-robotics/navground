import argparse

from . import echo, info, record_video, replay, run, run_rt, sample, list_plugins


def config_parser(parsers, name, module):
    parser = parsers.add_parser(name, help=module.description())
    module.init_parser(parser)
    parser.set_defaults(func=module._main)


def init_parser(parser: argparse.ArgumentParser) -> None:
    parsers = parser.add_subparsers(dest='cmd',
                                    title='Subcommands',
                                    metavar="{info,run,...}")
    config_parser(parsers, 'info', info)
    config_parser(parsers, 'echo', echo)
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


def main():
    ps = parser()
    args = ps.parse_args()
    if args.cmd is not None:
        args.func(args)
    else:
        print("Welcome to navground!\n")
        ps.print_help()
