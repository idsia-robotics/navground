=======
GridMap
=======

.. py:type:: Cell
   :canonical: typing.Annotated[numpy.typing.NDArray[numpy.int32], '[2, 1]']

   The two dimensional index of a GridMap.

.. py:type:: CellLike
   :canonical: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, '[2, 1]']

   Anything that can be interpreted as the two dimensional index of a GridMap.

.. py:type:: Map
   :canonical: typing.Annotated[numpy.typing.NDArray[numpy.uint8], '[m, n]']

   The two dimensional grey-scale map.

.. autoclass:: navground.core.GridMap
   :members:
   :show-inheritance:
