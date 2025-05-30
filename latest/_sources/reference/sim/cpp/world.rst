=====
World
=====

.. code-block:: cpp
   
   #include "navground/sim/world.h"

.. doxygentypedef:: navground::sim::BoundingBox

.. doxygenfunction:: navground::sim::bb_to_tuple
.. doxygenfunction:: navground::sim::bb_from_tuple
.. doxygenfunction:: navground::sim::envelop
.. doxygenfunction:: navground::sim::bb_set_max_x
.. doxygenfunction:: navground::sim::bb_set_max_y
.. doxygenfunction:: navground::sim::bb_set_min_x
.. doxygenfunction:: navground::sim::bb_set_min_y

.. doxygenstruct:: navground::sim::Entity
   :members:

.. doxygenstruct:: navground::sim::Obstacle
   :members:

.. doxygenstruct:: navground::sim::Wall
   :members:

.. doxygenclass:: navground::sim::World
   :members:
   

.. note::

   Class :cpp:class:`World` supports `dynamic attributes <attributes cpp>`.