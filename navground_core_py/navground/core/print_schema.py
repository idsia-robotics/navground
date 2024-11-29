import argparse
import sys
from typing import Any
from collections.abc import Callable

import yaml
from navground import core
from navground.core import command

Schema = dict[str, Any]

Schemas = dict[str, Callable[[], Schema]]
schemas: Schemas = {
    "core": core.schema.bundle,
}

Components = dict[str, type[Any]]
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
                        choices=kinds,
                        nargs='?')
    parser.add_argument(
        "--register",
        help=
        "Whether to generate the register schema instead of the base class schema",
        action='store_true')
    parser.add_argument(
        "--type",
        type=str,
        help=
        "If provided, generates the schema for the sub-class registered under this name"
    )


def init_parser(parser: argparse.ArgumentParser) -> None:
    init_parser_with_schemas(parser, 'core', schemas, components)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    init_parser(p)
    return p


def schema(arg: argparse.Namespace, schemas: Schemas,
           components: Components) -> None:
    if arg.kind not in schemas and arg.kind not in components:
        print(f"Unknown kind {arg.kind}", file=sys.stderr)
        sys.exit(1)
    if arg.kind in components:
        cls = components[arg.kind]
        if arg.register:
            schema = cls.register_schema()
        else:
            schema = cls.schema(True, arg.type)
    else:
        schema = schemas[arg.kind]()
    if not schema:
        print("Empty schema", file=sys.stderr)
        sys.exit(1)
    print(yaml.safe_dump(schema))


def _main(arg: argparse.Namespace) -> None:
    command._main(arg)
    schema(arg, schemas, components)


def main() -> None:
    _main(parser().parse_args())
