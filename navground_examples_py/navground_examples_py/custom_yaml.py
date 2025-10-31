from __future__ import annotations

import sys
import typing

import jsonschema
import yaml
from navground import core


class MyCustomBehavior(core.Behavior, name="Custom"):

    def __init__(self,
                 kinematics: core.Kinematics | None = None,
                 radius: float = 0) -> None:
        super().__init__(kinematics, radius)
        self._param = False
        self._readonly_param = 1

    def encode(self, value: dict[str, typing.Any]) -> dict[str, typing.Any]:
        value["param"] = self._param
        value["readonly_param"] = self._readonly_param
        return value

    def decode(self, value: dict[str, typing.Any]) -> None:
        self._param = value.get("param", False)

    @core.schema.register
    @staticmethod
    def custom_schema(node: dict[str, typing.Any]) -> None:
        node["properties"]['param'] = {'type': 'boolean', 'default': False}


def main() -> None:
    instance = {'type': 'Custom', 'param': True}
    behavior = core.Behavior.load(yaml.safe_dump(instance))
    if behavior:
        print("Behavior\n========")
        print(behavior.dump())
        print()
        schema = core.Behavior.schema(type="Custom")
        print("Schema\n======")
        print(yaml.safe_dump(schema))
        try:
            jsonschema.validate(instance=instance, schema=schema)
            print("Valid")
        except jsonschema.exceptions.ValidationError as e:
            print(f"Not valid: {e}", file=sys.stderr)
    else:
        print("Could not load behavior", file=sys.stderr)
