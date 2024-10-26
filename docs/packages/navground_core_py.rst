=================
navground_core_py
=================

This package provides Python bindings to the core library.

Dependencies
============

- :doc:`navground_core </packages/navground_core>` the C++ library
- `Python >= 3.7 <https://www.python.org>`_ the Python runtime
- `pybind11 <https://pybind11.readthedocs.io/en/stable/>`_ for binding the C++ library
- `This fork of pybind11_mkdoc <https://github.com/jeguzzi/pybind11_mkdoc/tree/rst>`_ to extract Python docstrings from the C++ code
- `NumPy <https://numpy.org>`_ pybind11 exposes Eigen objects using `numpy`


Python packages
===============

navground.core
--------------

A Python package with navigation algorithms, controllers, and kinematics, see the :doc:`API reference </reference/core/python/index>`.

To use the package

#. add the install path to ``PYTHONPATH``

#. import the package

   .. code-block:: python

      from navground import core

Libraries
==========

navground_core_py
-----------------

A C++ headers-only library with utilities for Python bindings for internal use.

Executables
===========

.. _info_py:
info
----

Lists registered components (behaviors, kinematics, and behavior modulations).
It is equivalent to the :ref:`C++ version <info>` but with additional components implemented in Python.


.. argparse::
   :module: navground.core.info
   :func: parser
   :prog: info
   :nodefault:
   :nodescription:


Example
~~~~~~~

.. code-block:: console

   $ info --properties
     
   Behaviors
   ---------
   Dummy
   HL
        aperture: 3.141592653589793 [double]
        barrier_angle: 1.5707963267948966 [double]
        epsilon: 0.0 [double]
        eta: 0.5 [double]
        resolution: 101 [int]
        tau: 0.125 [double]
   HRVO
        max_neighbors: 1000 [int]
        uncertainty_offset: 0.0 [double]
   ORCA
        effective_center: False [bool]
        max_neighbors: 1000 [int]
        static_time_horizon: 10.0 [double]
        time_horizon: 10.0 [double]
        treat_obstacles_as_agents: True [bool]
   PyDummy
        dummy: True [bool], deprecated synonyms: not_so_smart
        tired: False [bool]
   SocialForce
        c: 0.5 [double]
        phi: 1.75 [double]
        step_duration: 1.0 [double]
        tau: 0.5 [double]
        u_a: 10.0 [double]
        u_r: 0.2 [double]
        v_a: 2.1 [double]
        v_r: 0.3 [double]
   
   Kinematics
   ----------
   2WDiff
        wheel_axis: 0.0 [double]
   2WDiffDyn
        max_acceleration: 0.0 [double]
        moi: 1.0 [double]
        wheel_axis: 0.0 [double]
   4WOmni
        wheel_axis: 0.0 [double]
   Ahead
   Omni
   
   Behavior modulations
   --------------------
   LimitAcceleration
        max_acceleration: inf [double]
        max_angular_acceleration: inf [double]
   MotorPID
        k_d: 0.0 [double]
        k_i: 0.0 [double]
        k_p: 1.0 [double]
   Relaxation
        tau: 0.125 [double]


.. _echo_py:

echo
----

Load and then print a YAML representation of an object (behavior, kinematic, or behavior modulation). It is equivalent to the :ref:`C++ version <echo>` but with additional components implemented in Python.


.. argparse::
   :module: navground.core.echo
   :func: parser
   :prog: echo
   :nodefault:
   :nodescription:


Example
~~~~~~~

.. code-block:: console

   $ echo behavior "{type: PyDummy}"

   type: PyDummy
   dummy: true
   tired: false
   optimal_speed: 0
   optimal_angular_speed: 0
   rotation_tau: 0.5
   safety_margin: 0
   horizon: 5
   path_look_ahead: 1
   path_tau: 0.5
   radius: 0
   heading: velocity
   social_margin:
     modulation:
       type: constant
     default: 0


.. _plugins_py:

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
   Behaviors: Idle [C++]


navground.core
--------------

Instead of 

.. code-block:: console

   install/lib/navground_core_py/<command> [arguments]


you can call the subcommand (``info``) from Python, like

.. code-block:: console

   python -m navground.core [sub-command] [arguments]

