==========
Experiment
==========

Schema
^^^^^^

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/experiment
   title: Experiment
   type: object
   properties:
     time_step: number
     steps: integer
     runs: integer
     save_directory: string
     record_pose: bool
     record_twist: bool
     record_cmd: bool
     record_target: bool
     record_collisions: bool
     record_safety_violation: bool
     record_task_events: bool
     terminate_when_all_idle: bool
     scenario: {$ref: /schemas/scenario}
     name: string
   required: []

Example
^^^^^^^

.. code-block:: yaml

   time_step: 0.1
   steps: 1000
   runs: 10
   save_directory: data
   record_pose: true
   record_collisions: true
   terminate_when_all_idle: true
   scenario:
     type: Cross
     agent_margin: 0.125
     side: 6
     tolerance: 0.25
     walls:
       - [[-1.0, -1.0], [-1.0, 1.0]]
     obstacles:
       - 
         position: [2.0, 0.0]
         radius: 0.5
     groups:
       - behavior:
           type: HL
           horizon: 1
         kinematics:
           type: 2WDiff
           wheel_axis: 0.125
           max_speed: 0.25
         radius: 0.15
         control_period: 0.1
         number: 4

