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

.. ng-command-output:: info --properties
   :package: navground_core
   :ellipsis: 20

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

.. ng-command-output:: echo kinematics "{type: 2WDiff}"
   :package: navground_core
   :ellipsis: 20


.. _schema:

schema
-------

Print JSON-Schema of YAML-convertible navground core classes.

.. argparse::
   :module: navground.core.print_schema
   :func: parser
   :prog: schema
   :nodescription:
   :nodefault:

Example
~~~~~~~

.. ng-command-output:: schema core
   :package: navground_core
   :ellipsis: 20

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

.. .. ng-command-output:: plugins
..    :package: navground_core
..    :ellipsis: 20

.. code-block:: console

   $ plugins

   navground_examples
   ------------------
   Behaviors: Idle [C++]