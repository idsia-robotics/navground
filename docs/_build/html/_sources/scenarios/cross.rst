=====
Cross
=====


In this scenario, there are 4 target waypoints located at ``(-side/2, 0)``, ``(side/2, 0)``, ``(0, -side/2)``, and ``(0, side/2)``. Half of the agents are tasked to pendle between the two vertically aligned waypoints, and half between the horizontally aligned waypoints. The scenario tests how agents cross in the middle, where the 4 opposing flows meets. 

Initialization
==============

No extra agents or obstacles are created. Agents are uniformly initialized inside the area, avoiding that they overlap. Agents tasks are setup to make them pendle between opposing target waypoints.


Parameters
==========
	

:agent_margin: 

	the minimal distance between agents at initialization
	(float, default = ``0.1``).

:add_safety_to_agent_margin:

	Wheter to add the agent safety margin to the minimal distance between agents at initialization (float, bool = ``true``).

:side:

	the distance between opposing targets
	(float, default = ``2.0``).

:target_margin:

	the minimal distance between agent and target at initialization
	(float, default = ``0.5``).

:tolerance:

	how near to the target point before considering it as reached
	(float, default = ``0.25``).


Example
=======

.. video:: cross.mp4
	:loop:
	:width: 780

The simulation in the video can be reproduced with

.. code-block:: console

   $ run_rt src/navground/navground_demos/launch/cross.yaml  --factor 5 --background lightgray


where the experiment is configured like

.. code-block:: YAML

   steps: 3000
   time_step: 0.04
   save_directory: ''
   record_pose: true
   scenario:
     type: Cross
     agent_margin: 0.1
     side: 4
     target_margin: 0.1
     tolerance: 0.5
     groups:
       -
         type: thymio
         number: 20
         radius: 0.08
         control_period: 0.1
         speed_tolerance: 0.02
         kinematics:
           type: 2WDiff
           wheel_axis: 0.094
           max_speed: 0.166
         behavior:
           type: HL
           optimal_speed: 0.12
           horizon: 5.0
           safety_margin: 0.02
         state_estimation:
           type: Bounded
           range_of_view: 5.0



