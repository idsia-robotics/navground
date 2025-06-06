from __future__ import annotations

import argparse
import logging
import pathlib
import sys
from typing import Any

import yaml
from navground import core
from navground.core import command

kinds: list[str] = ["behavior", "behavior_modulation", "kinematics"]


def description() -> str:
    return "Validate YAML."


def init_parser_with_kinds(parser: argparse.ArgumentParser, version: str,
                           kinds: list[str]) -> None:
    command.init_parser(parser, version)
    parser.add_argument("kind",
                        type=str,
                        help="The kind of object to validate",
                        choices=kinds)
    parser.add_argument(
        "YAML",
        type=str,
        help="YAML string, or path to a YAML file, describing an experiment")


def init_parser(parser: argparse.ArgumentParser) -> None:
    init_parser_with_kinds(parser, core.get_build_info().version_string, kinds)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def get_schema(arg: argparse.Namespace) -> dict[str, Any]:
    if arg.kind not in kinds:
        logging.error(f"Unknown kind of object to validate: {arg.kind}")
        sys.exit(1)
    schema = core.schema.bundle()
    schema['$ref'] = arg.kind
    return schema


def validate(arg: argparse.Namespace,
             schema: dict[str, Any]) -> None:
    import jsonschema

    yaml_str = arg.YAML
    try:
        path = pathlib.Path(yaml_str)
        if path.exists():
            with open(path) as f:
                yaml_str = f.read()
    except Exception as e:
        logging.error(f"Failed to load file: {e}")
        sys.exit(1)

    instance = yaml.safe_load(yaml_str)
    try:
        jsonschema.validate(instance=instance, schema=schema)
    except Exception as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    validate(arg, get_schema(arg))


def main() -> None:
    _main(parser().parse_args())
