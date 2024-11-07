import argparse
import typing

from navground import core


def init_parser(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--no-plugins',
                        help="Do not load plugins",
                        action='store_true')


def _main(arg: argparse.Namespace,
          load_plugins: typing.Callable[[], None] = core.load_plugins) -> None:
    if not arg.no_plugins:
        load_plugins()
