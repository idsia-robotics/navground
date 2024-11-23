import typing

from ._navground import schema as core

Schema: typing.TypeAlias = dict[str, typing.Any]
SchemaModifier: typing.TypeAlias = typing.Callable[[Schema], None]


def register(modifier: SchemaModifier) -> staticmethod:
    fn = staticmethod(modifier)
    fn.__is_schema__ = True  # type: ignore
    return fn


def positive(schema: Schema) -> None:
    schema["minimum"] = 0


def strict_positive(schema: Schema) -> None:
    schema["exclusiveMinimum"] = 0


def longer_than(value: int) -> SchemaModifier:

    def f(schema: Schema) -> None:
        schema["minItems"] = value + 1

    return f


__all__ = ['core', 'register', 'positive', 'strict_positive', 'longer_than']
