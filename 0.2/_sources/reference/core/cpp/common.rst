========
Geometry
========

Two dimensional
===============

.. code-block:: cpp
   
   #include "navground/core/common.h"


.. doxygentypedef:: navground::core::Radians

.. doxygenenum:: navground::core::Frame

.. doxygentypedef:: navground::core::Vector2

.. doxygenfunction:: navground::core::orientation_of

.. doxygenfunction:: navground::core::normalize_angle

.. doxygenfunction:: navground::core::to_relative

.. doxygenfunction:: navground::core::to_absolute

.. doxygenfunction:: navground::core::to_relative_point

.. doxygenfunction:: navground::core::to_absolute_point

.. doxygenfunction:: navground::core::unit

.. doxygenfunction:: navground::core::rotate

.. doxygenfunction:: navground::core::clamp_norm

.. doxygenstruct:: navground::core::Pose2
   :members:

.. doxygenstruct:: navground::core::Twist2
   :members:


Three dimensional
=================

.. code-block:: cpp
   
   #include "navground/core/controller_3d.h"

.. doxygentypedef:: navground::core::Vector3

.. doxygenstruct:: navground::core::Pose3
   :members:

.. doxygenstruct:: navground::core::Twist3
   :members: