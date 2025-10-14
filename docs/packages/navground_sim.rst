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

.. ng-command-output:: info --properties
   :package: navground_sim
   :ellipsis: 20


.. _echo_sim:

echo
----

Load and then print a YAML representation of an object (behavior, kinematic, behavior modulation, state estimation, task, scenarios, agent, world, experiment, sampler).


.. argparse::
   :module: navground.sim.echo
   :func: parser
   :prog: echo
   :nodescription:
   :nodefault:


Example
~~~~~~~

.. ng-command-output:: echo scenario "{type: Corridor, agent_margin: 0.25, width: 2}"
   :package: navground_sim
   :ellipsis: 20


.. _schema_sim:

schema
-------

Print JSON-Schema of YAML-convertible navground sim classes.

.. argparse::
   :module: navground.sim.print_schema
   :func: parser
   :prog: schema
   :nodescription:
   :nodefault:

Example
~~~~~~~

.. ng-command-output:: schema sim
   :package: navground_sim
   :ellipsis: 20


.. _plugins_sim:

plugins
-------

Load and list plugins.

.. argparse::
   :module: navground.sim.list_plugins
   :func: parser
   :prog: plugins
   :nodescription:
   :nodefault:

Example
~~~~~~~

.. .. ng-command-output:: plugins
..    :package: navground_sim
..    :ellipsis: 20

.. code-block:: console

   $ plugins

   navground_demos
   ---------------
   Scenarios: ThymioDemo
   
   navground_examples
   ------------------
   Behaviors: Idle
   Scenarios: Empty
   

.. _sample:

sample
------

Samples a world from a scenario containing components implemented in C++, or from a sampler.


.. argparse::
   :module: navground.sim.sample
   :func: parser
   :prog: sample
   :nodescription:

Example (scenario)
~~~~~~~~~~~~~~~~~~

.. ng-command-output:: sample "{type: Antipodal, groups: [{number: 2}]}"
   :package: navground_sim
   :ellipsis: 20

Example (sampler)
~~~~~~~~~~~~~~~~~

.. ng-command-output:: sample "{sampler: uniform, from: 0, to: 10}" --type int --number 5
   :package: navground_sim
   :ellipsis: 20

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

.. ng-command-output:: run  "{save_directory: "/tmp", scenario: {type: Antipodal, groups: [{number: 20}]}}"
   :package: navground_sim
   :ellipsis: 20


.. note:: Although individual runs execute in a single thread, we can speed up experiments consisting of *multiple* runs by parallelizing them. Check out :ref:`the related guide <parallelize_guide>` to know more.

 .. _navground:

navground
---------

A command that contains all other commands of this package as sub-commands, installed in the binary directory. Using it, you can run

.. code-block:: console

   $ naground <command> [arguments]

instead of 

.. code-block:: console

   $ install/lib/navground_sim/<command> [arguments]

Example
~~~~~~~

.. command-output:: navground run --help  
   :ellipsis: 20
