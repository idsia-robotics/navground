======================
SVG and HTML rendering
======================

:module: :py:mod:`navground.sim.ui`

.. py:currentmodule:: navground.sim.ui

.. py:type:: Decorate
   :canonical: typing.Callable[[navground.sim.Entity], dict[str, str]]

   A function that returns a dictionary of SVG style attributes for an entity, like
   ``{"fill": "red"}``.

.. py:type:: Rect
   :canonical: tuple[numpy.ndarray, numpy.ndarray] | numpy.ndarray

   A rectangle defined by two points, at the bottom-left and top-right vertices.

.. autofunction:: svg_color

.. autofunction:: svg_for_world

.. autofunction:: html_for_world
