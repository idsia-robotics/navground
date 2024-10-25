import argparse

from navground import core


def init_parser(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--no-plugins',
                        help="Do not load plugins",
                        action='store_true')


def _main(arg: argparse.Namespace) -> None:
    if not arg.no_plugins:
        core.load_plugins()
