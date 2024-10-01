===========
Cross Torus
===========

In this scenario, agents follow a target direction (left, right, up, or down). Each direction is followed by one quarter of the agents.
The world boundaries are wrapped (like on a torus): when agents pass the border of a square lattice cell (of side ``side``), the are teleported to the opposite side. Agents perception take this wrapping also into account, possibly seeing multiple copies of the same agent on the lattice. Like ``Cross``, the scenario tests how 4 streams of agents cross.


Initialization
==============

No extra agents or obstacles are created. Agents are uniformly initialized inside the lattice cell, avoiding that they overlap.


Parameters
==========
	

:agent_margin: 

	the minimal distance between agents at initialization
	(float, default = ``0.1``).

:add_safety_to_agent_margin:

	Wheter to add the agent safety margin to the minimal distance between agents at initialization (float, bool = ``true``).

:side:

	the size of the squared lattice cell
	(float, default = ``2.0``).


Example
=======

.. video:: cross_torus.mp4
	:loop:
	:width: 780

The video has been recorded using

.. code-block:: console

   $ navground record_video install/navground_examples_yaml/experiment/cross_torus.yaml --factor 5

and a real time simulation can be visualized in a browser using

.. code-block:: console

   $ navground run_rt install/navground_examples_yaml/experiment/cross_torus.yaml --factor 5

where the experiment is configured like

.. code-block:: YAML

   steps: 1000
   time_step: 0.1
   save_directory: ''
   record_pose: true
   scenario:
     type: CrossTorus
     agent_margin: 0.1
     side: 4
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
