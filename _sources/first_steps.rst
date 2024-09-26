============
First Steps
============

Preparation
===========

If you built navground from source, you need to source the setup script of the workspace where you installed it.

.. tabs::

   .. tab:: bash

      .. code-block:: console

         source ./install/setup.bash

   .. tab:: zsh

      .. code-block:: console

         source ./install/setup.zsh

   .. tab:: Windows command shell

      .. code-block:: console

         install\setup.bat


Instead, if you installed navground using ``pip``, you don't need any setup but you are limited to the Python version of the commands (which is not a real limitation, as you Python commands are a superset of the C++ commands.)

Command Line Interface
======================

Navground has a modular architecture which users can extend by implementing new behaviors, kinematics, modulations, state estimation, tasks and scenarios. At installation, navground provides the implementation of several of these components so that you can start playing with them.
For instance, to list components available from C++, we can run

.. code-block:: console

   navground info


You can list the other sub-commands with: 

.. code-block:: console

   navground --help


The Python package has few more sub-commands:

.. code-block:: console

   navground_py --help

   usage: navground_py [-h] {info,run,run_rt,sample,record_video,replay}    ...
   
   positional arguments:
     {info,run,run_rt,sample,record_video,replay}
                           Subcommands
   
   options:
     -h, --help            show this help message and exit


.. note::

   You can also launch ``navground_py`` from Python too.

   .. code-block:: console

      python -m navground.sim --help

All commands are also available as standalone executables, installed in ``install/lib``:

.. tabs::

   .. tab:: macOS and Linux

      .. code-block:: console

         install/lib/<package>/<command>

   .. tab:: Windows

      .. code-block:: console

         install\Lib\<package>\<command>

and, if installed, from ROS 2

.. code-block:: console

   ros2 run <package> <command>

.. note::

   In the rest of the documentation, when describing commands, we omit ``ros2 run ...`` or the install path prefix and only specify the command to run, like 
   
   .. code-block:: console
   
      info

Next steps
==========

From here on, you may 

want to go through some tutorials
   start with :doc:`tutorials/tour`

want to have a look at some examples
   find them at :doc:`packages/navground_examples`, :doc:`packages/navground_examples_py`, and :doc:`packages/navground_demos`

want get familiar with the installed packages and their commands
   go to :doc:`packages/index`

want to check out the programming interfaces 
   have a look at :doc:`reference/index`




