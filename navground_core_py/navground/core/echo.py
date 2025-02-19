from __future__ import annotations

import argparse
import logging
import pathlib
import sys
from collections.abc import Callable
from typing import Any

from navground import core
from navground.core import command

from .utils import chdir

Echos = dict[str, Callable[[str], Any]]
echos: Echos = {
    "behavior": core.load_behavior,
    "modulation": core.load_behavior_modulation,
    "kinematics": core.load_kinematics
}


def description() -> str:
    return "Load an object from YAML and print its YAML representation."


def init_parser_with_echos(parser: argparse.ArgumentParser, version: str,
                           echos: Echos) -> None:
    command.init_parser(parser, version)
    kinds = ", ".join(echos.keys())
    parser.add_argument("kind",
                        type=str,
                        help=f"The kind of object to load: {kinds}")
    parser.add_argument(
        "YAML",
        type=str,
        help="YAML string, or path to a YAML file, describing an experiment")
    parser.add_argument(
        '--chdir',
        help=(
            "Whether to change working directory to the directory containing "
            "the file. Useful when the config contains relative paths."),
        action='store_true')


def init_parser(parser: argparse.ArgumentParser) -> None:
    init_parser_with_echos(parser, core.get_build_info().version_string, echos)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def echo(arg: argparse.Namespace,
         echos: Echos,
         dump: Callable[[Any], str] = core.dump) -> None:
    if arg.kind not in echos:
        logging.error(f"Unknown kind of object to load: {arg.kind}")
        sys.exit(1)
    yaml = arg.YAML
    wd: pathlib.Path | None = None
    try:
        path = pathlib.Path(yaml)
        if path.exists():
            with open(path) as f:
                yaml = f.read()
            if arg.chdir:
                wd = path.parent
    except Exception as e:
        logging.error(f"Failed to load file: {e}")
        sys.exit(1)
    with chdir(wd):
        obj = echos[arg.kind](yaml)
        if obj:
            print(dump(obj))
        else:
            logging.error(f"Failed to load {arg.kind}")
            sys.exit(1)


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    echo(arg, echos)


def main() -> None:
    _main(parser().parse_args())
