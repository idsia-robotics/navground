import argparse
import logging
import pathlib
import sys
from typing import Any, Callable, Dict

from navground import core
from navground.core import command

Echos = Dict[str, Callable[[str], Any]]
echos: Echos = {
    "behavior": core.load_behavior,
    "modulation": core.load_behavior_modulation,
    "kinematics": core.load_kinematics
}


def description() -> str:
    return "Load an object from YAML and print its YAML representation."


def init_parser(parser: argparse.ArgumentParser, echos: Echos = echos) -> None:
    command.init_parser(parser)
    kinds = ", ".join(echos.keys())
    parser.add_argument("kind",
                        type=str,
                        help=f"The kind of object to load: {kinds}")
    parser.add_argument(
        "YAML",
        type=str,
        help="YAML string, or path to a YAML file, describing an experiment")


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
    try:
        path = pathlib.Path(yaml)
        if path.exists():
            with open(path, 'r') as f:
                yaml = f.read()
    except Exception as e:
        logging.error(f"Failed to load file: {e}")
        sys.exit(1)

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
