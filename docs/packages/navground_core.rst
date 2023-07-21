==============
navground_core
==============

This package contains the core implementation of the navigation playground.

Dependencies
============

- `ament_cmake <https://index.ros.org/p/ament_cmake/#humble>`_ to build the package
- `Eigen3 >= 3.3 <https://eigen.tuxfamily.org>`_ for linear algebra
- `yaml-cpp <https://github.com/jbeder/yaml-cpp>`_ for reading and writing YAML snippets

Libraries
=========

navground_core
---------------

A shared C++ library with navigation algorithms, controllers, and kinematics, see the :doc:`API reference </reference/core/cpp/index>`.

To use the library in a C++ CMake project:

#. add the dependency in `CMakeLists.txt`

   .. code-block:: cmake

      find_package(navground_core REQUIRED)
      # if using ament
      # ament_target_dependencies((<MYTARGET> navground_core)
      # else
      target_link_libraries(<MYTARGET> PRIVATE navground_core::navground_core)
      # if it's a library
      # add_dependencies(<MYTARGET> navground_core::navground_core)

#. include the appropriate headers in your code

   .. code-block:: cpp

      #include "navground/core/behavior.h"


Executables
===========

.. _info:

info
----

Lists registered components (behaviors and kinematics).

.. code-block::
	
   usage: info [--help] [--version] [--behavior [BEHAVIOR]] [--kinematics [KINEMATICS]]

Named Arguments
~~~~~~~~~~~~~~~

-h, --help
   shows help message and exits 

-v, --version
   prints version information and exits 

--behavior
   selects behaviors

--kinematics
   selects kinematics

Example
~~~~~~~

.. code-block:: console

   $ info       
   Behaviors
   ---------
   Dummy
   HL
        aperture: 3.14159 [float]
        eta: 0.5 [float]
        resolution: 101 [int]
        tau: 0.125 [float]
   HRVO
   ORCA
        effective_center: 0 [bool]
        time_horizon: 10 [float]
   
   Kinematics
   ----------
   2WDiff
        wheel_axis: 0 [float]
   4WOmni
        wheel_axis: 0 [float]
   Ahead
   Omni
