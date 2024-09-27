======================
Command Line Interface
======================

Behind libraries for C++ and Python, navground provides commands.

.. warning:: 

   As explained in :doc:`first_steps`, if you built navground from source, you need to source the setup script of the workspace where you installed it to enable the commands. Instead, if you installed navground using ``pip``, you can call Python commands without any setup.

You can call a ``<command>`` in up to four different ways, depending on the installation type and on package that implements the command.

``navground{_py} <command>`` [1]
   ``navground`` exposes the commands implemented in :doc:`packages/navground_sim` in C++, while ``navground_py`` exposes the commands implemented in :doc:`packages/navground_sim_py` in Python. The Python commands are a superset of the C++ commands.
``python -m navground.{core|sim} <command>`` [2]
   ``navground.core`` exposes the commands implemented in :doc:`packages/navground_core_py`, while ``navground.sim`` the the commands implemented in :doc:`packages/navground_sim_py`, i.e., the same commands as through ``navground_py``.
``install/lib/<package>/<command>`` [3]
   stand-alone executables installed if you built from source.
``ros2 run <package> <command>`` [4]
   ROS 2 short-cut to run command ``install/lib/<package>/<command>``, available only if you have ROS.

For example, listing the installed components, can be done in three equivalent ways:

.. tabs::

   .. tab:: macOS and Linux

      .. code-block:: bash

         navground_py info
         python3 -m navground.sim info
         # only if you built from source 
         install/lib/navground_core_sim/info
         # only if you have ROS 
         ros2 run navground_core_sim info

   .. tab:: Windows

      .. code-block:: bash

         navground_py.exe info
         python -m navground.sim info
         # only if you built from source 
         install\Lib\navground_core_sim\info
         # only if you have ROS 
         ros2 run navground_core_sim info

.. note::

   In the rest of the documentation, when describing a <command>, we omit the prefix, only specifying the <command>, like 
   
   .. code-block:: console
   
      info

   for the previous example.

Commands
========

The following table summarizes all the available commands and how you can launch them.

.. table::
   :widths: auto

   +-------------------+---------------------+---------------------------------------------+
   | package           | commands            | how to run them                             |
   +===================+=====================+=============================================+
   | navground_core    | :ref:`info`         | ``install/lib/navground_core/<command>``    |
   |                   |                     |                                             |
   |                   |                     | ``ros2 run navground_core <command>``       |
   +-------------------+---------------------+---------------------------------------------+
   | navground_core_py | :ref:`info_py`      | ``install/lib/navground_core_py/<command>`` |
   |                   |                     |                                             |
   |                   |                     | ``ros2 run navground_core_py <command>``    |
   |                   |                     |                                             |
   |                   |                     | ``python -m navground.core <command>``      |
   +-------------------+---------------------+---------------------------------------------+
   | navground_sim     | :ref:`info_sim`     | ``install/lib/navground_sim_py/<command>``  |
   |                   |                     |                                             |
   |                   | :ref:`sample`       | ``ros2 run navground_sim <command>``        |
   |                   |                     |                                             |
   |                   | :ref:`run`          | ``navground <command>``                     |
   +-------------------+---------------------+---------------------------------------------+
   | navground_sim_py  | :ref:`info_sim_py`  | ``install/lib/navground_sim_py/<command>``  |
   |                   |                     |                                             |
   |                   | :ref:`sample_py`    | ``ros2 run navground_sim_py <command>``     |
   |                   |                     |                                             |
   |                   | :ref:`run_py`       | ``navground_py <command>``                  |
   |                   |                     |                                             |
   |                   | :ref:`run_rt`       | ``python -m navground.sim  <command>``      |
   |                   |                     |                                             |
   |                   | :ref:`record_video` |                                             | 
   |                   |                     |                                             |
   |                   | :ref:`replay`       |                                             |
   +-------------------+---------------------+---------------------------------------------+



