=================
Environment State
=================

Base class
==========

.. code-block:: cpp
   
   #include "navground/core/state.h"

.. doxygenstruct:: navground::core::EnvironmentState
    :members:


Geometric
=========

.. code-block:: cpp
   
   #include "navground/core/states/geometric.h"

.. doxygenstruct:: navground::core::LineSegment
    :members:

.. doxygenstruct:: navground::core::Disc
    :members:

.. doxygenstruct:: navground::core::Neighbor
    :members:

.. doxygenclass:: navground::core::GeometricState
    :members:


Sensing
=======

.. code-block:: cpp
   
   #include "navground/core/buffer.h"

.. doxygentypedef:: navground::core::BufferShape

.. doxygentypedef:: navground::core::BufferType

.. doxygentypedef:: navground::core::BufferData

.. doxygenstruct:: navground::core::BufferDescription
    :members:

.. doxygenclass:: navground::core::Buffer
    :members:

.. code-block:: cpp
   
   #include "navground/core/states/sensing.h"

.. doxygenclass:: navground::core::SensingState
    :members:
