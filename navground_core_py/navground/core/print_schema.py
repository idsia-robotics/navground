import argparse
import logging
import sys
from typing import Callable, Dict, Type

import yaml
from navground import core
from navground.core import command

Schemas = Dict[str, Callable[[], Dict]]
schemas: Schemas = {
    "core": core.schema.core,
}

Components = Dict[str, Type]
components: Components = {
    "behavior": core.Behavior,
    "behavior_modulation": core.BehaviorModulation,
    "kinematics": core.Kinematics
}


def description() -> str:
    return "Prints the YAML schema"


def init_parser_with_schemas(parser: argparse.ArgumentParser,
                             default_schema: str, schemas: Schemas,
                             components: Components) -> None:
    command.init_parser(parser)
    kinds = list(set(schemas.keys()) | set(components.keys()))
    parser.add_argument("kind",
                        type=str,
                        help="The target type of the scheme",
                        default=default_schema,
                        choices=kinds)
    parser.add_argument("--type",
                        type=str,
                        help="Registered component name",
                        default="")


def init_parser(parser: argparse.ArgumentParser) -> None:
    init_parser_with_schemas(parser, 'core', schemas, components)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def schema(arg: argparse.Namespace, schemas: Schemas,
           components: Components) -> None:
    if arg.kind not in schemas and arg.kind not in components:
        logging.error(f"Unknown kind of target: {arg.kind}")
        sys.exit(1)
    if arg.kind in components:
        cls = components[arg.kind]
        if arg.type:
            schema = cls.schema_of_type(arg.type)
        else:
            schema = cls.base_schema(True)
    else:
        schema = schemas[arg.kind]()
    print(yaml.safe_dump(schema))


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    schema(arg, schemas, components)


def main() -> None:
    _main(parser().parse_args())
