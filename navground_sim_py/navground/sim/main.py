import argparse

from . import info, record_video, replay, run, run_rt, sample


def config_parser(parsers, name, module):
    parser = parsers.add_parser(name)
    module.init_parser(parser)
    parser.set_defaults(func=module._main)


def main():
    parser = argparse.ArgumentParser(prog='navground')
    parsers = parser.add_subparsers(required=True, dest='cmd', help="Subcommands")
    config_parser(parsers, 'info', info)
    config_parser(parsers, 'run', run)
    config_parser(parsers, 'run_rt', run_rt)
    config_parser(parsers, 'sample', sample)
    config_parser(parsers, 'record_video', record_video)
    config_parser(parsers, 'replay', replay)
    args = parser.parse_args()
    args.func(args)
