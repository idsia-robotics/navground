from __future__ import annotations

import typing

from ._navground import bundle_schema as bundle

Schema: typing.TypeAlias = dict[str, typing.Any]
SchemaModifier: typing.TypeAlias = typing.Callable[[Schema], None]


def register(modifier: SchemaModifier) -> staticmethod[[Schema], None]:
    """
    Register a custom JSON-schema for a class

    Use it as:

    >>> class MyClass(Class, name=...):
    >>>
    >>> @schema.register
    >>> def modifier(schema: dict[str, typing.Any]) -> None:
    >>>     ...

    :param      modifier:  A modifier

    :returns:   The registered static method
    :rtype:     staticmethod
    """
    fn = staticmethod(modifier)
    fn.__is_schema__ = True  # type: ignore[attr-defined]
    return fn


def positive(schema: Schema) -> None:
    schema["minimum"] = 0


def strict_positive(schema: Schema) -> None:
    schema["exclusiveMinimum"] = 0


def longer_than(value: int) -> SchemaModifier:

    def f(schema: Schema) -> None:
        schema["minItems"] = value + 1

    return f


__all__ = ['bundle', 'register', 'positive', 'strict_positive', 'longer_than']
