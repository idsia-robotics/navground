from collections.abc import Callable, Collection
from typing import (Any, Literal, TypeAlias, TypeVar, cast, get_args,
                    get_origin, get_type_hints)

import numpy
from navground.core import uses_doubles
from navground.core.schema import SchemaModifier

FloatType: type[numpy.floating[Any]] = numpy.float64 if uses_doubles() else numpy.float32
# Type aliases cannot be dynamically evaluated ...
# else we would set the type to FloatType instead of numpy.float64
Vector2: TypeAlias = numpy.ndarray[tuple[Literal[2]], numpy.dtype[numpy.float64]]
Vector2Like: TypeAlias = Vector2 | tuple[float, float] | list[float]

ScalarPropertyField: TypeAlias = bool | int | float | str | Vector2
ListPropertyField: TypeAlias = list[bool] | list[int] | list[float] | list[
    str] | list[Vector2]
PropertyField: TypeAlias = ScalarPropertyField | ListPropertyField

T = TypeVar('T', bound=Any)


def _convert(getter: Callable[..., Any],
             value: Any) -> tuple[PropertyField, type[ScalarPropertyField]]:
    type_hint = get_type_hints(getter).get('return', None)
    type_: type[Any]
    item_type: type[Any] | None
    if type_hint:
        if type_hint == Vector2:
            type_ = Vector2
            item_type = None
        else:
            args = get_args(type_hint)
            if args:
                if len(args) != 1:
                    raise TypeError(
                        f"Invalid generic annotation: too may args {type_hint}"
                    )
                item_type = args[0]
                type_ = get_origin(type_hint)
            else:
                type_ = type_hint
                item_type = None
    else:
        type_ = type(value)
        item_type = None
    if type_ is tuple:
        type_ = list
    if type_ is list and item_type is None:
        try:
            item_type = type(value[0])
        except Exception as e:
            raise TypeError(
                f"Default value {value} should be an non-empty sequence"
            ) from e
    if type_ is list:
        return cast(ListPropertyField,
                    [_convert_scalar(item_type, x)
                     for x in value]), cast(type[ScalarPropertyField],
                                            item_type)
    return _convert_scalar(type_, value), type_


def _convert_scalar(type_: Any, value: Any) -> ScalarPropertyField:
    if type_ not in (bool, int, float, str, Vector2):
        raise TypeError(f"Unsupported type {type_}")

    if type_ == Vector2:
        try:
            if len(value) == 2:
                return numpy.asarray(value, dtype=float)
            else:
                raise ValueError(
                    f"Unsupported value with length {len(value)} != 2 for type Vector2"
                )
        except Exception as e:
            raise TypeError(
                f"Unsupported value {value} for type {type_}") from e
    if type_ is not type(value) and type_ not in (float, int) and not (
            type(value) in (float, int) and type_ in (float, int)):
        raise TypeError(
            f"Implicit conversion of {value} to {type_} is not permitted")
    try:
        return cast(ScalarPropertyField, type_(value))
    except Exception as e:
        raise TypeError(f"Unsupported value {value} for type {type_}") from e


def register(
    default_value: PropertyField,
    description: str = "",
    schema: SchemaModifier | None = None,
    deprecated_names: Collection[str] = tuple()
) -> Callable[[T], T]:
    """
    A decorator to register a property.
    It must be used below the ``@property`` decorator.

    For example, the following code adds a boolean valued
    registered property to a registered sub-class ``C`` of class ``T``:

    .. code-block:: python

        class C(T, name="C"):

            def __init__(self):
                super().__init__()
                self._my_field = True

            @property
            @register(True, "...")
            def my_property(self) -> bool:
                return self._my_field;

            @my_property.setter
            def my_property(self, value: bool) -> None:
                self._my_field = value;


    :param default_value:
        The default value of the property
        when the object is initialized

    :param description: The description of the property

    :param schema: A optional schema to add validity constrains to the property

    :param deprecated_names: A list of alternative deprecated names
    """

    def g(f: T) -> T:
        try:
            value, scalar_type = _convert(f, default_value)
            f.__default_value__ = value
            if scalar_type in (bool, int, float, str):
                f.__scalar_value__ = scalar_type()  # type: ignore[call-arg]
            else:
                f.__scalar_value__ = numpy.zeros(2, dtype=FloatType)
        except Exception as e:
            raise ValueError('Default property value not valid') from e
        f.__desc__ = description
        f.__deprecated_names__ = deprecated_names
        f.__schema__ = schema
        return f

    return g
