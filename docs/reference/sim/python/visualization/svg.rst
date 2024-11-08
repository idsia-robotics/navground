======================
SVG and HTML rendering
======================

:module: :py:mod:`navground.sim.ui`

.. py:currentmodule:: navground.sim.ui

.. autoclass:: Decorate

   A function that returns a dictionary of SVG style attributes for an entity, like
   ``{"fill": "red"}``.

.. py:class:: Rect
   :module: navground.sim.ui

   A typealias to ``tuple[numpy.ndarray, numpy.ndarray] | numpy.ndarray``: a rectangle defined by two points, at the bottom-left and top-right vertices.

.. autofunction:: svg_color

.. autofunction:: svg_for_world

.. autofunction:: html_for_world
