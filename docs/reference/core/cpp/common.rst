========
Geometry
========

Two dimensional
===============

.. code-block:: cpp
   
   #include "navground/core/common.h"

.. doxygentypedef:: Radians

.. doxygenvariable:: navground::core::PI
.. doxygenvariable:: navground::core::TWO_PI
.. doxygenvariable:: navground::core::HALF_PI

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