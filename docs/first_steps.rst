===========
First Steps
===========

You have installed navground, congratulations! 
Let us now check that it is working properly.

If you built navground from source, you need to source the workspace.

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         $ . install/setup.zsh

   .. tab:: Linux

      .. code-block:: console
         
         $ . install/setup.bash


   .. tab:: Windows

      .. code-block:: console
        
         $ install\setup.bat

Call a similar command if you installed a binary release from Github:

.. tabs::

   .. tab:: macOS (zhs)

      .. code-block:: console

         $ . /opt/install/setup.zsh

   .. tab:: Linux (bash)

      .. code-block:: console
         
         $ . /opt/install/setup.bash


   .. tab:: Windows (command prompt)

      .. code-block:: console
        
         $ "C:\Program Files\navground\setup.bat"

   .. tab:: Windows (power shell)

      First, you may need to increase the privilege of the shell, for instance by running

      .. code-block:: console

         $ Set-ExecutionPolicy -ExecutionPolicy Unrestricted -Scope Process -Force

      Then execute the script

      .. code-block:: console
        
         $ & "C:\Program Files\install\setup.ps1"

Once configured, run ``navground_py``, which is installed for any type of installations. 

.. tabs::

   .. tab:: Linux & macOS

      .. code-block:: console

         $ navground_py

   .. tab:: Windows

      .. code-block:: console

         $ navground_py.exe

You should get a welcome message with few sub-commands to run.


.. command-output:: navground_py


Navground has a modular architecture that users can extend by implementing new behaviors, kinematics, modulations, state estimation, tasks, and scenarios. At installation, navground provides several of these components to play with: calling ``info`` will list them.

.. tabs::

   .. tab:: Linux & macOS

      .. code-block:: console

         $ navground_py info

   .. tab:: Windows

      .. code-block:: console

         $ navground_py.exe info


.. program-output:: navground_py info --no-plugins

Next steps
==========

From here on, you may 

want to get familiar with the command line interface
   read :doc:`cli`

want to go through some tutorials
   start with :doc:`tutorials/tour`

want to have a look at some examples
   find them at :doc:`packages/navground_examples`, :doc:`packages/navground_examples_py`, and :doc:`packages/navground_demos`

want get familiar with the installed packages and their commands
   go to :doc:`packages/index`

want to discover which components are installed
   have a look at :doc:`components/index`

want to get guided through extending navground with new components
   have a look at :doc:`guides/extend/index`

want to check out the programming interfaces 
   have a look at :doc:`reference/index`


