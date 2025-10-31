======
Target
======

.. py:currentmodule:: navground.core

.. py:type:: Curve
   :canonical: collections.abc.Callable[[float], tuple[Vector2, typing.SupportsFloat, typing.SupportsFloat]]
   
   The parametrization of a (G2) curve by length.

   The function returns position, tangential orientation and curvature at a coordinate, like:

   .. code-block:: python

      def curve(coordinate: SupportsFloat) -> tuple[Vector2, float, float]]:
         ...
         return (position, orientation, curvature)

.. py:type:: Projection
   :canonical: collections.abc.Callable[[Vector2Like, typing.SupportsFloat, typing.SupportsFloat], float]

   The projection of a curve parametrized by length.

   The function finds the coordinate in ``[begin, end]`` where 
   ``curve(coordinate)`` is nearest to the given point point, like:

   .. code-block:: python

      def projection(point: Vector2Like, begin: SupportsFloat, end: SupportsFloat) -> float: 
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
