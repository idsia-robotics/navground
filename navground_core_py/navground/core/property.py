from typing import (Any, Callable, List, Literal, Tuple, TypeAlias, Union,
                    TypeVar, Type)

from .schema import SchemaModifier
import numpy

# TODO(Jerome): how to define an alias that depends on `uses_doubles`?
Vector2: TypeAlias = numpy.ndarray[tuple[Literal[2]],
                                   numpy.dtype[numpy.float64]]
Vector2Like: TypeAlias = Vector2 | tuple[float, float] | list[float]

ScalarPropertyField = Union[bool, int, float, str, Vector2]
PropertyField = Union[ScalarPropertyField, List[bool], List[int], List[float],
                      List[str], List[Vector2]]

T = TypeVar('T', bound=Any)


def _convert(type_: Any, value: Any) -> Tuple[PropertyField, Type]:
    if isinstance(type_, str):
        try:
            type_ = eval(type_)
        except Exception:
            type_ = None

    item_type = None
    try:
        # decompose X[y] in X and y
        item_type, *_ = type_.__args__
        type_ = type_.__origin__
    except:
        pass
    if not item_type:
        try:
            item_type = type(value[0])
        except:
            pass
    if not type_:
        type_ = type(value)
    if item_type is list:
        item_type = Vector2
    if type_ in (list, tuple, List, Tuple):
        return [_convert_scalar(item_type, x)
                for x in value], item_type  # type: ignore
    return _convert_scalar(type_, value), type_


def _convert_scalar(type_: Any, value: Any) -> ScalarPropertyField:
    try:
        type_ = type_.__origin__
    except:
        pass

    if type_ not in (bool, int, float, str, numpy.ndarray):
        raise TypeError(f"Unsupported type {type_}")

    if type_ is numpy.ndarray:
        try:
            if len(value) == 2:
                return numpy.asarray(value, dtype=float)
            else:
                raise ValueError(
                    f"Unsupported value with length {len(value)} != 2 for type Vector2"
                )
        except:
            raise TypeError(f"Unsupported value {value} for type {type_}")

    if type_ is not type(value) and type_ not in (
            float, int) and type(value) not in (float, int):
        raise TypeError(
            f"Implicit conversion of {value} to {type_} is not permitted")
    try:
        return type_(value)
    except:
        pass
    raise TypeError(f"Unsupported value {value} for type {type_}")


def register(default_value: PropertyField,
             description: str = "",
             schema: SchemaModifier | None = None,
             deprecated_names: list[str] = []) -> Callable[[T], T]:
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
        return_type = f.__annotations__.get('return')
        try:
            value, scalar_type = _convert(return_type, default_value)
            f.__default_value__ = value
            if scalar_type in (bool, int, float, str):
                f.__scalar_value__ = scalar_type()
            else:
                f.__scalar_value__ = numpy.zeros(2)
        except Exception as e:
            raise ValueError(f'Default property value not valid: {e}')
        f.__desc__ = description
        f.__deprecated_names__ = deprecated_names
        f.__schema__ = schema
        return f

    return g
