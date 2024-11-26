=============
Visualization
=============

.. py:module:: navground.sim.ui
   :synopsis: Visualization

Visualization related functionality are implemented in sub-package
:py:mod:`navground.sim.ui` and not imported in the top-level package 
to minimize dependencies.

SVG and HTML rendering
======================

:module: :py:mod:`navground.sim.ui`

.. py:currentmodule:: navground.sim.ui

.. autoclass:: Decorate

   A function that returns a dictionary of SVG style attributes for an entity, like
   ``{"fill": "red"}``.

.. autoclass:: Rect

   A rectangle defined by bottom-left and top-right points.

.. autofunction:: svg_color

.. autofunction:: svg_for_world

.. autofunction:: html_for_world

Rasterized rendering
====================

:module: :py:mod:`navground.sim.ui.render`

.. py:module:: navground.sim.ui.render
   :synopsis: Rasterized Rendering

   Rasterizes SVG using `cairosvg <https://cairosvg.org>`_.

.. autofunction:: image_for_world

.. autofunction:: png_for_world

.. autofunction:: pdf_for_world


Video
=====

:module: :py:mod:`navground.sim.ui.video`

.. py:module:: navground.sim.ui.video
   :synopsis: Video utils
   
   Uses `moviepy <https://zulko.github.io/moviepy/>`_.

.. autofunction:: navground.sim.ui.video.record_video

.. autofunction:: navground.sim.ui.video.record_video_from_run

.. autofunction:: navground.sim.ui.video.display_video

.. autofunction:: navground.sim.ui.video.display_video_from_run


Notebook helpers
================

:module: :py:mod:`navground.sim.notebook`

.. py:module:: navground.sim.notebook
   :synopsis: Jupyter utils

   Helpers to display simulations in Jupyter notebooks.

.. autofunction:: display_in_notebook

.. autofunction:: notebook_view

.. autofunction:: run_in_notebook


Matplotlib helpers
==================

:module: :py:mod:`navground.sim.pyplot_helpers`

.. py:module:: navground.sim.pyplot_helpers
   :synopsis: Matplotlib utils

   Helpers to plot simulations using `matplotlib <https://matplotlib.org>`_

.. autofunction:: plot_agent

.. autofunction:: plot_world

.. autofunction:: plot_trajectory

.. autofunction:: plot_run


Real-time
=========

:module:  :py:mod:`navground.sim.ui`

.. py:currentmodule:: navground.sim.ui

.. autoclass:: WebUI
    :members:
    :exclude-members: __new__
