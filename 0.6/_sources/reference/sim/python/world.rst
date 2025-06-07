=====
World
=====

.. autoclass:: navground.sim.BoundingBox
   :members:

.. autoclass:: navground.sim.Entity
   :members:
   :exclude-members: __init__

.. autoclass:: navground.sim.Obstacle
   :members:

.. autoclass:: navground.sim.Wall
   :members:

.. autoclass:: navground.sim.World
   :members:
   :exclude-members: __new__
   :inherited-members:

   .. py:attribute:: render_kwargs
      :type: dict[str, typing.Any]
   
      Rendering configuration: specified fields override
      :py:data:`navground.sim.ui.render_default_config` for this world
      in rendering functions like :py:func:`navground.sim.ui.svg_for_world`.


.. note::

   Class :py:class:`World` supports `dynamic attributes <attributes python>`.