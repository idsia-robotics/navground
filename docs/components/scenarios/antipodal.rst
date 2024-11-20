=========
Antipodal
=========

In this scenario, all agents are initially equally spaced along a circle, each oriented towards the circle center. They are all tasked to reach the antipodal (i.e., opposing) point along the circle. The scenario tests how the agents cross in the middle. Some behavior let a coordinated swirling pattern emerge which is very efficient.

Initialization
==============

No extra agents or obstacles are created. All agents are placed on a circle and their task is set. 


Parameters
==========

:orientation_noise:

	the standard deviation of the normal distribution added to the orientation at initialization. Set to zero to orient the agents perfectly towards their target. (float, default = ``0.0``)
	

:position_noise: 

	the standard deviation of the normal distribution independently added to the x and y coordinates at initialization. Set to zero to place the agents on a perfect circle.
	(float, default = ``0.0``)

:radius:

	the radius of the circle where agents are initially placed
	(float, default = ``1.0``)

:shuffle:

	wheter the agents should be randomly shuffled before placing them on the circle
	(bool, default = ``false``)

:tolerance:

	how near to the target point before considering it as reached
	(float, default = ``0.1``)


Example
=======

.. video:: antipodal.mp4
	:loop:
	:width: 780

The video has been recorded using

.. code-block:: console

   $ navground record_video install/navground_examples_yaml/experiment/antipodal.yaml --factor 5

and a real time simulation can be visualized in a browser using

.. code-block:: console

   $ navground run_rt install/navground_examples_yaml/experiment/antipodal.yaml --factor 5


where the experiment is configured like

.. code-block:: YAML

   steps: 1000
   time_step: 0.1
   save_directory: ''
   record_pose: true
   scenario:
     type: Antipodal
     position_noise: 0.005
     orientation_noise: 0.01
     radius: 2
     tolerance: 0.2
     groups:
       -
         type: thymio
         number: 20
         radius: 0.1
         control_period: 0.1
         speed_tolerance: 0.01
         kinematics:
           type: 2WDiff
           wheel_axis: 0.094
           max_speed: 0.166
         behavior:
           type: HL
           optimal_speed: 0.08
           horizon: 10.0
           safety_margin: 0.01
         state_estimation:
           type: Bounded
           range: 10.0






