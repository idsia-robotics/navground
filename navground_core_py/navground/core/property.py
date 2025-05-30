from __future__ import annotations

from collections.abc import Callable, Collection
from typing import (Any, Literal, TypeAlias, TypeVar, cast, get_args,
                    get_origin, get_type_hints)

import numpy
from navground.core import uses_doubles
from navground.core.schema import SchemaModifier

FloatType: type[
    numpy.floating[Any]] = numpy.float64 if uses_doubles() else numpy.float32
# Type aliases cannot be dynamically evaluated ...
# else we would set the type to FloatType instead of numpy.float64
Vector2: TypeAlias = numpy.ndarray[tuple[Literal[2]],
                                   numpy.dtype[numpy.float64]]
Vector2Like: TypeAlias = Vector2 | tuple[float, float] | list[float]

ScalarPropertyField: TypeAlias = bool | int | float | str | Vector2
ListPropertyField: TypeAlias = list[bool] | list[int] | list[float] | list[
    str] | list[Vector2]
PropertyField: TypeAlias = ScalarPropertyField | ListPropertyField

T = TypeVar('T', bound=Any)


def _get_scalar_type_name(type_: type[ScalarPropertyField]) -> str:
    if type_ in (bool, int, float, str):
        return type_.__name__
    return 'vector'


def _is_list(scalar_type_name: str, value: Any) -> bool:
    if scalar_type_name == 'vector':
        if (type(value) in (list, tuple) and len(value) == 2
                and all(type(x) in (bool, int, float) for x in value)):
            return False
    return type(value) in (list, tuple)


def _get_type(getter: Callable[..., Any],
              value: Any) -> tuple[type[ScalarPropertyField], bool]:
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
        return cast(type[ScalarPropertyField], item_type), True
    return type_, False


def register(
    default_value: PropertyField,
    description: str = "",
    schema: SchemaModifier | None = None,
    deprecated_names: Collection[str] = tuple(),
    scalar_type_name: str = '',
    type_safe: bool = False,
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

    .. note::

       The setter of registered properties is wrapped to guarantee
       that is is called only with arguments of the property type
       (e.g., py:type:`bool` in the example above).

       In the example above, trying to set the property using

       .. code-block:: python

          c = C()
          c.my_property = 'a'

       or

       .. code-block:: python

          c.set('my_property', 'a'')

       has no effect.

       You can disable this argument coercion by setting the
       environment variable ``NAVGROUND_DISABLE_PY_PROPERTY_COERCION``
       (before navground is initialized) or by passing ``type_safe=True``.


    :param default_value:
        The default value of the property
        when the object is initialized

    :param description: The description of the property

    :param schema: A optional schema to add validity constrains to the property

    :param deprecated_names: A list of alternative deprecated names

    :param scalar_type_name: The property scalar type name:
                             one of "int", "float", "bool", "str, "vector".
                             If not provided it will infer it from
                             the type hints and/or the default value type.

    :param type_safe: Whether the setter is type safe and therefore does not require
                      checking/coercing the argument.
    """

    def g(f: T) -> T:
        if scalar_type_name:
            f.__scalar_type__ = scalar_type_name
            f.__is_list__ = _is_list(scalar_type_name, default_value)
        else:
            try:
                scalar_type, f.__is_list__ = _get_type(f, default_value)
                f.__scalar_type__ = _get_scalar_type_name(scalar_type)
            except Exception as e:
                raise ValueError('Default property value not valid') from e
        f.__desc__ = description
        f.__deprecated_names__ = deprecated_names
        f.__schema__ = schema
        f.__type_safe__ = type_safe
        f.__default_value__ = default_value
        return f

    return g
