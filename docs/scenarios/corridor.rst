.. _corridor:

========
Corridor
========


In this scenario, half of the agents need to travel towards one end of a straight corridor, and the other half towards the other end. The two ends are wrapped together, i.e., agents exiting from on side are reintroduced on the other side. State estimation and collisions both conform to this lattice. 
The scenario tests opposing flows of agents. Some behavior let the agents spontaneously organize in lanes of opposing flow.

Initialization
==============

Two walls are added along the corridor. No extra agents are created. Agents are uniformly initialized inside the corridor, avoiding that they overlap. Agents navigation target directions are set.


Parameters
==========
	

:agent_margin: 

	the minimal distance between agents at initialization.
	(float, default = ``0.1``)

:add_safety_to_agent_margin:

	Wheter to add the agent safety margin to the minimal distance between agents at initialization. (float, bool = ``true``)

:length:

	the length of the corridor. 
	(float, default = ``10.0``)

:width:

	the width of the corridor.
	(float, default = ``1.0``)


Example
=======

.. video:: corridor.mp4
	:loop:
	:width: 780

The simulation in the video can be reproduced with

.. code-block:: console

   $ run_rt src/navground/navground_demos/launch/corridor.yaml  --factor 5 --background lightgray


where the experiment is configured like

.. code-block:: YAML

   steps: 3000
   time_step: 0.04
   save_directory: ''
   record_pose: true
   scenario:
     type: Corridor
     length: 8.0
     width: 0.6
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
           range: 5.0
