=====
World
=====

.. code-block:: cpp
   
   #include "navground/sim/world.h"

BoundingBox
===========

.. doxygentypedef:: navground::sim::BoundingBox

.. doxygenfunction:: navground::sim::bb_to_tuple
.. doxygenfunction:: navground::sim::bb_from_tuple
.. doxygenfunction:: navground::sim::envelop
.. doxygenfunction:: navground::sim::bb_set_max_x
.. doxygenfunction:: navground::sim::bb_set_max_y
.. doxygenfunction:: navground::sim::bb_set_min_x
.. doxygenfunction:: navground::sim::bb_set_min_y

Entities
========

.. doxygenstruct:: navground::sim::Entity
   :members:

.. doxygenstruct:: navground::sim::Obstacle
   :members:

.. doxygenstruct:: navground::sim::Wall
   :members:

World
=====

.. doxygenclass:: navground::sim::World
   :members:
   

.. note::

   Class :cpp:class:`World` supports :ref:`dynamic attributes <attributes cpp>`.