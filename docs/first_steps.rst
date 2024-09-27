===========
First Steps
===========

If you have installed navground, congratulations! 

Let us check that it is working too.

If you built navground from source, you need to source the workspace.

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         . install/setup.zsh

   .. tab:: Linux

      .. code-block:: console
         
         . install/setup.bash


   .. tab:: Windows

      .. code-block:: console
        
         install\setup.bat

Run ``navground_py``, which will have been installed by all types of installations. 

.. tabs::

   .. tab:: Linux & macOS

      .. code-block:: console

         navground_py

   .. tab:: Windows

      .. code-block:: console

         navground_py.exe

You should get a welcome message and few sub-commands to run.

.. code-block:: console

   Welcome to navground!
   
   usage: navground_py [-h] {info,run,...} ...
   
   options:
     -h, --help      show this help message and exit
   
   Subcommands:
     {info,run,...}
       info          Lists registered components.
       run           Runs an experiment using the Python interpreter
       run_rt        Runs an experiment using the Python interpreter in real-time.
       sample        Samples a world from a scenario.
       record_video  Make video from an experiment using the Python interpreter.
       replay        Replay an experiment in real-time.


Navground has a modular architecture which users can extend by implementing new behaviors, kinematics, modulations, state estimation, tasks and scenarios. At installation, navground provides the implementation of several of these components to play with. Calling ``info`` will list them

.. tabs::

   .. tab:: Linux & macOS

      .. code-block:: console

         navground_py info

   .. tab:: Windows

      .. code-block:: console

         navground_py.exe info


.. code-block:: console

   Behaviors
   ---------
   Dummy, HL, HRVO, ORCA, PyDummy, SocialForce
   
   Kinematics
   ----------
   2WDiff, 2WDiffDyn, 4WOmni, Ahead, Omni
   
   Modulations
   -----------
   LimitAcceleration, MotorPID, Relaxation
   
   State Estimations
   -----------------
   Boundary, Bounded, Combination, Discs, Lidar, pyLidar
   
   Tasks
   -----
   Direction, Waypoints
   
   Scenarios
   ---------
   Antipodal, Corridor, Cross, CrossTorus, Simple

Next steps
==========

From here on, you may 

want to go get familiar with the command line interface
   read :doc:`cli`

want to go through some tutorials
   start with :doc:`tutorials/tour`

want to have a look at some examples
   find them at :doc:`packages/navground_examples`, :doc:`packages/navground_examples_py`, and :doc:`packages/navground_demos`

want get familiar with the installed packages and their commands
   go to :doc:`packages/index`

want to check out the programming interfaces 
   have a look at :doc:`reference/index`


