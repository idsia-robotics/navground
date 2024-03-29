============
navground_py
============

This package provides Python bindings to the core library.

Dependencies
============

- `ament_cmake <https://index.ros.org/p/ament_cmake/#humble>`_ to build the package
- :doc:`navground_core </packages/navground_core>` the C++ library
- `Python >= 3.7 <https://www.python.org>`_ the Python runtime
- `pybind11 <https://pybind11.readthedocs.io/en/stable/>`_ for binding the C++ library
- `NumPy <https://numpy.org>`_ pybind11 exposes Eigen objects using `numpy`
- `This fork of pybind11_mkdoc <https://github.com/jeguzzi/pybind11_mkdoc/tree/rst>`_ to extract Python docstrings from the C++ code.


Python packages
===============

navground.core
--------------

A Python package with navigation algorithms, controllers, and kinematics, see the :doc:`API reference </reference/core/python/index>`.

To use the package

#. add the install path to `PYTHONPATH`

#. import the packge

   .. code-block:: python

      from navground import core

Libraries
==========

navground_py
------------

A C++ headers-only library with utilities for Python bindings for internal use.

Executables
===========

info
----

Lists registered components (behaviors and kinematics).
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

   $ info       
   Behaviors
   =========
   
   - Dummy
   - HL
        aperture: 3.1415927410125732 [float]
        eta: 0.5 [float]
        resolution: 101 [int]
        tau: 0.125 [float]
   - HRVO
   - ORCA
        effective_center: False [bool]
        time_horizon: 10.0 [float]
   - PyDummy
        dummy: True [bool]
        tired: False [bool]
   - SocialForce
   
   Kinematics
   ==========
   
   - 2WDiff
        wheel_axis: 0.0 [float]
   - 4WOmni
        wheel_axis: 0.0 [float]
   - Ahead
   - Omni