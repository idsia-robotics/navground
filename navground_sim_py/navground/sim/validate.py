from __future__ import annotations

import argparse
import logging
import sys
from typing import Any

from navground import core, sim
from navground.core import command, validate

kinds: list[str] = validate.kinds + [
    "state_estimation", "task", "agent", "world", "scenario", "experiment",
    "sampler"
]

description = validate.description


def init_parser(parser: argparse.ArgumentParser) -> None:
    validate.init_parser_with_kinds(parser,
                                    sim.get_build_info().version_string, kinds)
    parser.add_argument("--type",
                        type=str,
                        help="The sampled type",
                        default='',
                        choices=[
                            'bool', 'int', 'float', 'str', 'vector', '[bool]',
                            '[int]', '[float]', '[str]', '[vector]'
                        ])


def parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    init_parser(parser)
    return parser


scalar_schemas_prefixes = {
    'int': 'number',
    'float': 'number',
    'str': 'string',
    'bool': 'boolean',
    'vector': 'vector2'
}

scalar_names = {
    'int': 'integer',
    'float': 'number',
    'str': 'string',
    'bool': 'boolean',
    'vector': 'vector2'
}


def get_schema(arg: argparse.Namespace) -> dict[str, Any]:
    if arg.kind not in kinds:
        logging.error(f"Unknown kind of object to validate: {arg.kind}")
        sys.exit(1)
    schema = sim.schema.bundle()
    if arg.kind == 'sampler':
        scalar_type, is_list = core.get_scalar_type_name(arg.type)
        scalar_schema_prefix = scalar_schemas_prefixes.get(scalar_type, '')
        scalar_name = scalar_names.get(scalar_type, '')
        if not scalar_schema_prefix or not scalar_name:
            raise ValueError(f"Unknown type {arg.type}")
        scalar = {'$ref' if scalar_name == 'vector2' else 'type': scalar_name}
        schema['$defs']['of'] = {'$dynamicAnchor': 'T'}
        if is_list:
            schema['$ref'] = 'list_sampler'
            schema['$defs']['of'].update({'type': "array", 'items': scalar})
        else:
            schema['$ref'] = f'{scalar_schema_prefix}_sampler'
            schema['$defs']['of'].update(scalar)
        print(schema['$ref'])
        print(schema['$defs']['of'])
    else:
        schema['$ref'] = arg.kind
    return schema


def _main(arg: argparse.Namespace) -> None:
    command._main(arg, sim.load_plugins)
    validate.validate(arg, get_schema(arg))


def main() -> None:
    arg = parser().parse_args()
    _main(arg)
