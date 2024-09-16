================
navground_sim_py
================

This package provides Python bindings to the simulation library and utilities to display the simulation.


Dependencies
============

- `ament_cmake <https://index.ros.org/p/ament_cmake/#humble>`_ to build the package
- :doc:`navground_core_py </packages/navground_core_py>` the core python package
- :doc:`navground_sim </packages/navground_sim>` the simulation library
- `pybind11 <https://pybind11.readthedocs.io/en/stable/>`_ for binding the C++ library
- `NumPy <https://numpy.org>`_ pybind11 exposes Eigen objects using `numpy`
- `This fork of pybind11_mkdoc <https://github.com/jeguzzi/pybind11_mkdoc/tree/rst>`_ to extract Python docstrings from the C++ code.


Python packages
===============

navground.sim
-------------

A Python package to perform navigation simulations; see the :doc:`API reference </reference/sim/python/index>`.

To use the package

#. add the install path to ``PYTHONPATH``

#. import the packge

   .. code-block:: python

      from navground import sim

Executables
===========

.. _navground_main:

navground
---------

Convenience script that group together all the other executables under the same command name


.. argparse::
   :module: navground.sim.main
   :func: parser
   :prog: navground
   :nodescription:
   :nodefault:


Example
~~~~~~~

.. code-block:: console

   $ navground --help       
   usage: navground [-h] {info,run,run_rt,sample,record_video,replay} ...
   
   positional arguments:
     {info,run,run_rt,sample,record_video,replay} Subcommands
   
   options:
     -h, --help            show this help message and exit


.. _info_sim_py:

info
----

Lists registered components (behaviors, kinematics, state estimations, tasks, and scenarios) implemented in C++ or Python.


.. argparse::
   :module: navground.sim.info
   :func: parser
   :prog: info
   :nodescription:
   :nodefault:


Example
~~~~~~~

.. code-block:: console

   $ info       
   Behaviors
   ---------
   Dummy
   HL
        aperture: 3.1415927410125732 [float]
        eta: 0.5 [float]
        resolution: 101 [int]
        tau: 0.125 [float]
   HRVO
   ORCA
        effective_center: False [bool]
        time_horizon: 10.0 [float]
   PyDummy
        dummy: True [bool]
        tired: False [bool]
   SocialForce
   
   Kinematics
   ----------
   2WDiff
        wheel_axis: 0.0 [float]
   4WOmni
        wheel_axis: 0.0 [float]
   Ahead
   Omni
   
   State estimations
   -----------------
   Dummy
   HL
        aperture: 3.1415927410125732 [float]
        eta: 0.5 [float]
        resolution: 101 [int]
        tau: 0.125 [float]
   HRVO
   ORCA
        effective_center: False [bool]
        time_horizon: 10.0 [float]
   PyDummy
        dummy: True [bool]
        tired: False [bool]
   SocialForce
   
   Tasks
   -----
   2WDiff
        wheel_axis: 0.0 [float]
   4WOmni
        wheel_axis: 0.0 [float]
   Ahead
   Omni
   
   Scenarios
   ---------
   2WDiff
        wheel_axis: 0.0 [float]
   4WOmni
        wheel_axis: 0.0 [float]
   Ahead
   Omni


.. _sample_py:

sample
------

Samples a world from a scenario with components implemented in C++ or Python.


.. argparse::
   :module: navground.sim.sample
   :func: parser
   :prog: sample
   :nodescription:

Example
~~~~~~~

.. code-block:: console

   $ sample "{type: Antipodal, groups: [{number: 2}]}"
   obstacles:
     []
   walls:
     []
   agents:
     - task:
         type: Waypoints
         loop: false
         tolerance: 0.100000001
         waypoints:
           -
             - -1
             - -0
       position:
         - 1
         - 0
       orientation: 3.14159274
       velocity:
         - 0
         - 0
       angular_speed: 0
       radius: 0
       control_period: 0
       type: ""
       id: 0
       uid: 0
     - task:
         type: Waypoints
         loop: false
         tolerance: 0.100000001
         waypoints:
           -
             - 1
             - 8.74227766e-08
       position:
         - -1
         - -8.74227766e-08
       orientation: 6.28318548
       velocity:
         - 0
         - 0
       angular_speed: 0
       radius: 0
       control_period: 0
       type: ""
       id: 0
       uid: 1


.. _run_py:

run
---

Run an experiment using a Python interpreter. It may be slightly slower than the C++ implementation, but has access to components implemented in Python.

.. argparse::
   :module: navground.sim.run
   :func: parser
   :prog: run
   :nodescription:

If the experiment is recording data, it will create a directory named ``<experiment_name>_<experiment_hash>_<datestamp>`` with

- an HDF5 file `data.h5`` with data recorded during the experiment,
- a YAML file `experiment.yaml` with the configuration of the experiment. 

Example
~~~~~~~

.. code-block:: console

   $ run  "{save_directory: ".", scenario: {type: Antipodal, groups: [{number: 20}]}}"
   Duration: 0.0120453 s
   Saved to: "./experiment_3784746994027959661_2023-07-07_16-13-36/data.h5"      


.. note::

    Although individual runs run in a single thread, we can speed up experiments consisting of *multiple* runs by parallelizing them. Check out :ref:`the related guide <parallelize_guide>` to know more.

.. _run_rt:

run_rt
------

Run an experiment using Python in real time. You can visualize the world in a browser view.


.. argparse::
   :module: navground.sim.run_rt
   :func: parser
   :prog: run_rt
   :nodescription:


Example
~~~~~~~

.. code-block:: console

   $ run_rt experiment.yaml --factor 5.0


record video
------------

Record a video from an experiment.


.. argparse::
   :module: navground.sim.record_video
   :func: parser
   :prog: record_video
   :nodescription:

Example
~~~~~~~

.. code-block:: console

   $ record_video experiment.yaml video.mp4 --factor 5.0

replay
------

Replays an experiment in real-time. You can visualize the world in a browser view, similarly to run_rt_ but for recorded experiment, or create a video from it.


.. argparse::
   :module: navground.sim.replay
   :func: parser
   :prog: replay
   :nodescription:

Example
~~~~~~~

.. code-block:: console

   $ replay ./experiment_3784746994027959661_2023-07-07_16-13-36/data.h5 --factor 10
