============
First Steps
============

To use navground you need first to source the setup script of the workspace where you installed it.

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

Navground has a modular architecture which user can extend by implementing new behaviors, kinematics, state estimation, tasks and scenarios. At installation, ``navground`` provides the implementation of several of these components so that you can start playing with them.
For instance, to list components available from C++, we can run

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         install/lib/navground_sim/info

   .. tab:: Linux

      .. code-block:: console

         install/lib/navground_sim/info

   .. tab:: Windows

      .. code-block:: console

         install\Lib/navground_sim\info.exe  
   
From here on, you may 

want to go through some tutorials
   start with :doc:`tutorials/tour`

want to have a look at some examples
   find them at :doc:`packages/navground_examples`, :doc:`packages/navground_examples_py`, and :doc:`packages/navground_demos`

want get familiar with the installed packages and learn their command line
   go to :doc:`packages/index`

want to check out the programming interfaces 
   have a look at :doc:`reference/index`




