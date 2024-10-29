==============
navground_core
==============

This package contains the core implementation of the navigation playground.

Dependencies
============

- `Eigen3 >= 3.3 <https://eigen.tuxfamily.org>`_ for linear algebra
- `yaml-cpp <https://github.com/jbeder/yaml-cpp>`_ for reading and writing YAML snippets
- `arparse <https://github.com/p-ranav/argparse>`_ for CLI

Libraries
=========

navground_core
---------------

A C++ library with navigation algorithms, controllers, and kinematics, see the :doc:`API reference </reference/core/cpp/index>`.

To use the library in a C++ CMake project:

#. add the dependency in ``CMakeLists.txt``

   .. code-block:: cmake

      find_package(navground_core REQUIRED)
      # if using ament
      # ament_target_dependencies((<MYTARGET> navground_core)
      # else
      target_link_libraries(<MYTARGET> PRIVATE navground_core::navground_core)

#. include the appropriate headers in your code

   .. code-block:: cpp

      #include "navground/core/behavior.h"


Executables
===========

.. _info:

info
----

Lists registered components (behaviors, kinematics, and behavior modulations) implemented in C++.


.. argparse::
   :module: navground.core.info
   :func: parser
   :prog: info
   :nodescription:
   :nodefault:

Example
~~~~~~~

.. code-block:: console

   $ info --properties

   Behaviors
   ---------
   Dummy
   HL
        aperture: 3.14159 (double)
        barrier_angle: 1.5708 (double)
        epsilon: 0 (double)
        eta: 0.5 (double)
        resolution: 101 (int)
        tau: 0.125 (double)
   HRVO
        max_neighbors: 1000 (int)
        uncertainty_offset: 0 (double)
   ORCA
        effective_center: 0 (bool)
        max_neighbors: 1000 (int)
        static_time_horizon: 10 (double)
        time_horizon: 10 (double)
        treat_obstacles_as_agents: 1 (bool)
   
   Kinematics
   ----------
   2WDiff
        wheel_axis: 0 (double)
   2WDiffDyn
        max_acceleration: 0 (double)
        moi: 1 (double)
        wheel_axis: 0 (double)
   4WOmni
        wheel_axis: 0 (double)
   Ahead
   Omni
   
   Modulations
   -----------
   LimitAcceleration
        max_acceleration: inf (double)
        max_angular_acceleration: inf (double)
   MotorPID
        k_d: 0 (double)
        k_i: 0 (double)
        k_p: 1 (double)
   Relaxation
        tau: 0.125 (double)

.. _echo:

echo
----

Load and then print a YAML representation of an object (behavior, kinematic, or behavior modulation).

.. argparse::
   :module: navground.core.echo
   :func: parser
   :prog: echo
   :nodescription:
   :nodefault:

Example
~~~~~~~

.. code-block:: console

   $ echo kinematics "{type: 2WDiff}"

   type: 2WDiff
   max_backward_speed: 0
   max_forward_speed: .inf
   wheel_axis: 0
   max_speed: .inf
   max_angular_speed: .inf


.. _plugins:

plugins
-------

Load and list plugins.

.. argparse::
   :module: navground.core.list_plugins
   :func: parser
   :prog: plugins
   :nodescription:
   :nodefault:

Example
~~~~~~~

.. code-block:: console

   $ plugins

   navground_examples
   ------------------
   Behaviors: Idle
