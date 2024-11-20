======================
Command Line Interface
======================

Behind libraries for C++ and Python, navground provides commands.

.. warning:: 

   As explained in :doc:`first_steps`, if you built navground from source, you need to source the setup script of the workspace where you installed it to enable the commands. Instead, if you installed navground using ``pip``, you can call Python commands without any setup.

You can call a ``command`` in up to four different ways, depending on the type of installation and on which package implements the command.

``navground{_py} <command>`` [1]
   ``navground`` exposes the commands implemented in :doc:`packages/navground_sim` in C++, while ``navground_py`` exposes the commands implemented in :doc:`packages/navground_sim_py` in Python. The Python commands are a superset of the C++ commands.
``python -m navground.{core|sim} <command>`` [2]
   ``navground.core`` exposes the commands implemented in :doc:`packages/navground_core_py`, while ``navground.sim`` the the commands implemented in :doc:`packages/navground_sim_py`, i.e., the same commands as through ``navground_py``.
``install/lib/<package>/<command>`` [3]
   stand-alone executables installed if you built from source.
``ros2 run <package> <command>`` [4]
   ROS 2 short-cut to run command ``install/lib/<package>/<command>``, available only if you have ROS.

For example, calling the command that lists installed components available for simulating in Python, can be done in four equivalent ways:

.. tabs::

   .. tab:: macOS and Linux

      .. code-block:: bash

         navground_py info
         python3 -m navground.sim info
         # only if you built from source 
         install/lib/navground_sim_py/info
         # only if you have ROS 
         ros2 run navground_sim_py info

   .. tab:: Windows

      .. code-block:: bash

         navground_py.exe info
         python -m navground.sim info
         # only if you built from source 
         install\Lib\navground_sim_py\info
         # only if you have ROS 
         ros2 run navground_sim_py info

.. note::

   In the rest of the documentation, when describing a ``command``, we omit the prefix, only specifying the command like ``info``,
   for the previous example, instead of ``navground_py info`` and so on.

Commands
========

The following table summarizes all the available commands and how you can launch them.

.. table::
   :widths: auto

   +-------------------+------------------------+---------------------------------------------+
   |      package      |        commands        |               how to run them               |
   +===================+========================+=============================================+
   | navground_core    | :ref:`info`            | ``install/lib/navground_core/<command>``    |
   |                   |                        |                                             |
   |                   | :ref:`echo`            | ``ros2 run navground_core <command>``       |
   |                   |                        |                                             |
   |                   | :ref:`schema`          |                                             |
   |                   |                        |                                             |
   |                   | :ref:`plugins`         |                                             |
   +-------------------+------------------------+---------------------------------------------+
   | navground_core_py | :ref:`info_py`         | ``install/lib/navground_core_py/<command>`` |
   |                   |                        |                                             |
   |                   | :ref:`echo_py`         | ``ros2 run navground_core_py <command>``    |
   |                   |                        |                                             |
   |                   | :ref:`schema_py`       | ``python -m navground.core <command>``      |
   |                   |                        |                                             |
   |                   | :ref:`validate_py`     |                                             |
   |                   |                        |                                             |
   |                   | :ref:`plugins_py`      |                                             |
   +-------------------+------------------------+---------------------------------------------+
   | navground_sim     | :ref:`info_sim`        | ``install/lib/navground_sim_py/<command>``  |
   |                   |                        |                                             |
   |                   | :ref:`echo_sim`        | ``ros2 run navground_sim <command>``        |
   |                   |                        |                                             |
   |                   | :ref:`schema_sim`      | ``navground <command>``                     |
   |                   |                        |                                             |
   |                   | :ref:`plugins`         |                                             |
   |                   |                        |                                             |
   |                   | :ref:`sample`          |                                             |
   |                   |                        |                                             |
   |                   | :ref:`run`             |                                             |
   +-------------------+------------------------+---------------------------------------------+
   | navground_sim_py  | :ref:`info_sim_py`     | ``install/lib/navground_sim_py/<command>``  |
   |                   |                        |                                             |
   |                   | :ref:`echo_sim_py`     | ``ros2 run navground_sim_py <command>``     |
   |                   |                        |                                             |
   |                   | :ref:`schema_sim_py`   | ``navground_py <command>``                  |
   |                   |                        |                                             |
   |                   | :ref:`validate_sim_py` | ``python -m navground.sim  <command>``      |
   |                   |                        |                                             |
   |                   | :ref:`plugins_sim_py`  |                                             |
   |                   |                        |                                             |
   |                   | :ref:`sample_py`       |                                             |
   |                   |                        |                                             |
   |                   | :ref:`run_py`          |                                             |
   |                   |                        |                                             |
   |                   | :ref:`run_rt`          |                                             |
   |                   |                        |                                             |
   |                   | :ref:`record_video`    |                                             |
   |                   |                        |                                             |
   |                   | :ref:`replay`          |                                             |
   +-------------------+------------------------+---------------------------------------------+

