=============
navground_sim
=============

This package extend navigation playground with all you need to perform simulations and record data.

Dependencies
============

- :doc:`navground_core </packages/navground_core>` the core library
- `GEOS <https://libgeos.org>`_ for computational geometry
- `HighFive <https://github.com/BlueBrain/HighFive>`_ to write HDF5 files

Libraries
=========

navground_sim
-------------

A C++ library to perform navigation simulations; see the :doc:`API reference </reference/sim/cpp/index>`.

To use the library in a C++ CMake project:

#. add the dependency in ``CMakeLists.txt``

   .. code-block:: cmake

      find_package(navground_sim REQUIRED)
      # if using ament
      # ament_target_dependencies(<MYTARGET> navground_sim)
      # else
      target_link_libraries(<MYTARGET> PRIVATE navground_sim::navground_sim)

#. include the appropriate headers in your code

   .. code-block:: cpp

      #include "navground/sim/world.h"

Executables
===========

.. _info_sim:

info
----

Lists registered components (behaviors, kinematics, behavior modulations, state estimations, tasks, and scenarios) implemented in C++.


.. argparse::
   :module: navground.sim.info
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
   
   Scenarios
   ---------
   Antipodal
        orientation_noise: 0 (double)
        position_noise: 0 (double)
        radius: 1 (double)
        shuffle: 0 (bool)
        tolerance: 0.1 (double)
   Corridor
        add_safety_to_agent_margin: 1 (bool)
        agent_margin: 0.1 (double)
        length: 10 (double)
        width: 1 (double)
   Cross
        add_safety_to_agent_margin: 1 (bool)
        agent_margin: 0.1 (float)
        side: 2 (float)
        target_margin: 0.5 (float)
        tolerance: 0.25 (float)
   CrossTorus
        add_safety_to_agent_margin: 1 (bool)
        agent_margin: 0.1 (float)
        side: 2 (float)
   Simple
   
   State Estimations
   -----------------
   Boundary
        max_x: inf (double)
        max_y: inf (double)
        min_x: -inf (double)
        min_y: -inf (double)
        range: 1 (double)
   Bounded
        range: 1 (double), deprecated synonyms: range_of_view 
        update_static_obstacles: 0 (bool)
   Combination
   Discs
        include_valid: 1 (bool)
        max_id: 0 (int)
        max_radius: 0 (double)
        max_speed: 0 (double)
        number: 1 (int)
        range: 1 (double)
        use_nearest_point: 1 (bool)
   Lidar
        field_of_view: 6.28319 (double)
        range: 1 (double)
        resolution: 100 (int)
        start_angle: -3.14159 (double)
   
   Tasks
   -----
   Direction
        direction: [1, 0] (Vector2)
   Waypoints
        loop: 1 (bool)
        random: 0 (bool)
        tolerance: 1 (double)
        waypoints: [] (std::vector<Vector2>)


.. _sample:

sample
------

Samples a world from a scenario containing components implemented in C++.


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


.. _run:

run
---

Run an experiment limited to components implemented in C++.

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

   Although individual runs execute in a single thread, we can speed up experiments consisting of *multiple* runs by parallelizing them. Check out :ref:`the related guide <parallelize_guide>` to know more.


 .. _navground:

navground
---------

A command that contains all other commands of this package as sub-commands, installed in the binary directory. Using it, you can run

.. code-block:: console

   naground <command> [arguments]

instead of 

.. code-block:: console

   install/lib/navground_sim/<command> [arguments]

Example
~~~~~~~

.. code-block:: console

   $ navground run --help   

   Usage: run [--help] [--version] [--tqdm] [--run_index VAR] [--runs    VAR] [--threads VAR] [--processes VAR] YAML
   
   Runs an experiment.
   
   Positional arguments:
     YAML           YAML string, or path to a YAML file, describing an    experiment 
   
   Optional arguments:
     -h, --help     shows help message and exits 
     -v, --version  prints version information and exits 
     --tqdm         Display tqdm bar 
     --run_index    Will overwrite the experiment own run_index if    positive. [nargs=0..1] [default: -1]
     --runs         Will overwrite the experiment own runs if positive. [   nargs=0..1] [default: -1]
     --threads      Number of threads [nargs=0..1] [default: 1]
     --processes    Number of processes [only supported by run_py] [   nargs=0..1] [default: 1]   




