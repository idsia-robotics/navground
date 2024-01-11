=============
navground_sim
=============

This package extend navigation playground with all you need to perform simulations and record data.

Dependencies
============

- `ament_cmake <https://index.ros.org/p/ament_cmake/#humble>`_ to build the package
- :doc:`navground_core </packages/navground_core>` the core library
- :doc:`navground_py </packages/navground_py>` the core python package
- `GEOS <https://libgeos.org>`_ for computational geometry
- `HighFive <https://github.com/BlueBrain/HighFive>`_ to write HDF5 files

Libraries
=========

navground_sim
-------------

A shared C++ library to perform navigation simulations; see the :doc:`API reference </reference/sim/cpp/index>`.

To use the library in a C++ CMake project:

#. add the dependency in ``CMakeLists.txt``

   .. code-block:: cmake

      find_package(navground_sim REQUIRED)
      # if using ament
      # ament_target_dependencies((<MYTARGET> navground_sim)
      # else
      target_link_libraries(<MYTARGET> PRIVATE navground_sim::navground_sim)
      # if it's a library
      # add_dependencies(<MYTARGET> navground_sim::navground_sim)

#. include the appropriate headers in your code

   .. code-block:: cpp

      #include "navground/sim/world.h"


Python packages
===============

navground.sim
--------------

A Python package to perform navigation simulations; see the :doc:`API reference </reference/sim/python/index>`.

To use the package

#. add the install path to ``PYTHONPATH``

#. import the packge

   .. code-block:: python

      from navground import sim

Executables
===========

.. _info_sim:

info and info_py
----------------

Lists registered components (behaviors, kinematics, state estimations, tasks, and scenarios).
``info`` is limited to components implemented in C++, while ``info_py`` accesses components implemented in Python too.


.. argparse::
   :module: navground.sim.list
   :func: parser
   :prog: info|info_py
   :nodescription:
   :nodefault:


Example
~~~~~~~

.. code-block:: console

   $ info_py       
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


.. _sample:

sample and sample_py
--------------------

Samples a world from a scenario. ``sample_py`` uses a Python interpreter to access components implemented in Python too, while ``sample`` is limited to components implemented in C++.


.. argparse::
   :module: navground.sim.sample
   :func: parser
   :prog: sample|sample_py
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


.. _run:

run and run_py
--------------

Run an experiment. ``run_py`` uses a Python interpreter: it is slightly slower but has access to components implemented in Python too, while ``run`` is limited to components implemented in C++.

.. argparse::
   :module: navground.sim.run
   :func: parser
   :prog: run|run_py
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

    Although individual runs in a single thread, we can speed up experiments consisting of *multiple* runs by parallelizing them. Check out :ref:`the related guide <parallelize_guide>` to know more.

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
   :module: navground.sim.run_rt
   :func: parser
   :prog: run_rt
   :nodescription:

Example
~~~~~~~

.. code-block:: console

   $ replay ./experiment_3784746994027959661_2023-07-07_16-13-36/data.h5 --factor 10
