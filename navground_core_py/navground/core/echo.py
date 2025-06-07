from __future__ import annotations

import argparse
import logging
import pathlib
import sys
from collections.abc import Callable, Iterable
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


def init_parser_with_kinds(parser: argparse.ArgumentParser, version: str,
                           kinds: Iterable[str]) -> None:
    command.init_parser(parser, version)
    parser.add_argument("kind",
                        type=str,
                        help=f"The kind of object to load: {', '.join(kinds)}")
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
    init_parser_with_kinds(parser,
                           core.get_build_info().version_string, echos.keys())


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def load(arg: argparse.Namespace, yaml: str) -> Any:
    if arg.kind not in echos:
        logging.error(f"Unknown kind of object to load: {arg.kind}")
        sys.exit(1)
    return echos[arg.kind](yaml)


def echo(arg: argparse.Namespace,
         load_fn: Callable[[argparse.Namespace, str], Any] = load,
         dump: Callable[[Any], str] = core.dump) -> None:
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
        obj = load_fn(arg, yaml)
        if obj:
            print(dump(obj))
        else:
            logging.error(f"Failed to load {arg.kind}")
            sys.exit(1)


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    echo(arg)


def main() -> None:
    _main(parser().parse_args())
