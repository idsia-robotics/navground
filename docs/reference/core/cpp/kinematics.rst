==========
Kinematics
==========

.. code-block:: cpp
   
   #include "navground/core/kinematics.h"

Abstract classes
================

Kinematics base class
---------------------

.. doxygenclass:: navground::core::Kinematics
   :members:

Wheeled kinematics based class
------------------------------

.. doxygentypedef:: navground::core::WheelSpeeds

.. doxygenclass:: navground::core::WheeledKinematics
   :members:

Omnidirectional
===============

.. doxygenclass:: navground::core::OmnidirectionalKinematics
   :members:

Ahead
=====

.. doxygenclass:: navground::core::AheadKinematics
   :members:

Two wheels differential drive
=============================

.. doxygenclass:: navground::core::TwoWheelsDifferentialDriveKinematics
   :members:


Two wheels differential drive with dynamic constraints
======================================================

.. doxygenclass:: navground::core::DynamicTwoWheelsDifferentialDriveKinematics
   :members:


Four wheels omnidirectional drive
=================================

.. doxygenclass:: navground::core::FourWheelsOmniDriveKinematics
   :members: