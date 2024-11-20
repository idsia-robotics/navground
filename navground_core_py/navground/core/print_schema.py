import argparse
import logging
import sys
from typing import Callable, Dict

import yaml
from navground import core
from navground.core import command

Schemas = Dict[str, Callable[[], Dict]]
schemas: Schemas = {
    "core": core.schema,
    "behavior": core.Behavior.base_schema,
    "behavior_modulation": core.BehaviorModulation.base_schema,
    "kinematics": core.Kinematics.base_schema,
    "behavior_register": core.Behavior.register_schema,
    "behavior_modulation_register": core.BehaviorModulation.register_schema,
    "kinematics_register": core.Kinematics.register_schema,
}


def description() -> str:
    return "Prints the YAML schema"


def init_parser_with_schemas(parser: argparse.ArgumentParser, default_schema: str,
                             schemas: Schemas) -> None:
    command.init_parser(parser)
    parser.add_argument("kind",
                        type=str,
                        help="The target type of the scheme",
                        default=default_schema,
                        choices=list(schemas.keys()))


def init_parser(parser: argparse.ArgumentParser) -> None:
    init_parser_with_schemas(parser, 'core', schemas)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def schema(arg: argparse.Namespace, schemas: Schemas) -> None:
    if arg.kind not in schemas:
        logging.error(f"Unknown kind of target: {arg.kind}")
        sys.exit(1)
    schema = schemas[arg.kind]()
    print(yaml.safe_dump(schema))


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    schema(arg, schemas)


def main() -> None:
    _main(parser().parse_args())
