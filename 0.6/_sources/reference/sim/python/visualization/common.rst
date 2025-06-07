=============
Configuration
=============

:module: :py:mod:`navground.sim.ui`

.. py:currentmodule:: navground.sim.ui

.. py:type:: Decorate
   :canonical: typing.Callable[[navground.sim.Entity], dict[str, str]]

   A function that returns a dictionary of SVG style attributes for an entity, like
   ``{"fill": "red"}``.

.. py:type:: Rect
   :canonical: tuple[numpy.ndarray, numpy.ndarray] | numpy.ndarray

   A rectangle defined by two points, at the bottom-left and top-right vertices.

.. autoclass:: RenderConfig
   :members: 
   :exclude-members: __new__, __init__

.. autoattribute:: navground.sim.ui.render_default_config

.. autofunction:: svg_color

.. note:: 

   There are three ways to configure parameters for rendering functions:

   1. provide an optional argument to the function;
   2. set a world-specific option in :py:attr:`navground.sim.World.render_kwargs`;
   3. change the default configuration :py:data:`navground.sim.ui.render_default_config`.
