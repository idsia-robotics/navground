======
Target
======

.. py:currentmodule:: navground.core

.. py:type:: Curve
   :canonical: typing.Callable[[float], Vector2, float, float]]
   
   The parametrization of a (G2) curve by length.

   The function returns position, tangential orientation and curvature at a coordinate, like:

   .. code-block:: python

      def curve(coordinate: float) -> tuple[Vector2, float, float]]:
         ...
         return (position, orientation, curvature)

.. py:type:: Projection
   :canonical: typing.Callable[[Vector2, float, float], float]

   The projection of a curve parametrized by length.

   The function finds the coordinate in ``[begin, end]`` where 
   ``curve(coordinate)`` is nearest to the given point point, like:

   .. code-block:: python

      def projection(point: Vector2, begin: float, end: float) -> float: 
         ...
         return coordinate


.. autoclass:: Path
   :members:
   :undoc-members:
   :exclude-members: __new__

.. autoclass:: Target
   :members:
   :undoc-members:
   :exclude-members: __new__
