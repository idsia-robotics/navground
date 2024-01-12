=============
Visualization
=============

Visualization related functionality are provided in sub package
:module:  :py:mod:`navground.sim.ui` and not imported in the top-level package to minimize dependencies.


Display a simulated world
=========================

:module:  :py:mod:`navground.sim.ui`

.. autofunction:: navground.sim.ui.svg_for_world

.. autofunction:: navground.sim.ui.html_for_world

.. autofunction:: navground.sim.ui.notebook_view


:module:  :py:mod:`navground.sim.ui.render`

Render SVGs using ``cairosvg``.

.. autofunction:: navground.sim.ui.render.image_for_world

.. autofunction:: navground.sim.ui.render.png_for_world

.. autofunction:: navground.sim.ui.render.pdf_for_world


Synchronize HTML clients with a simulation 
==========================================

:module:  :py:mod:`navground.sim.ui`

.. autoclass:: navground.sim.ui.WebUI
    :members:
    :exclude-members: __new__

Video
=====

:module:  :py:mod:`navground.sim.ui.video`

Requires ``moviepy``.

.. autofunction:: navground.sim.ui.video.record_video

.. autofunction:: navground.sim.ui.video.record_video_from_run

.. autofunction:: navground.sim.ui.video.display_video

.. autofunction:: navground.sim.ui.video.display_video_from_run