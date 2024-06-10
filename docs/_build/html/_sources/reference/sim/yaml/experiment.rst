.. _experiment yaml:

==========
Experiment
==========

Schema
^^^^^^

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/record_neighbors
   title: RecordNeighborsConfig
   type: object
   properties:
     enabled: bool
     number: integer
     relative: bool
   required: []

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/record_neighbors
   title: RecordSensingConfig
   type: object
   properties:
     name: string
     sensor: {$ref: /schemas/state_estimation}
     agent_indices:    
      type: array
      items:
        type: integer
   required: []

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
     record_deadlocks: bool
     record_efficacy: bool
     record_neighbors: {$ref: /schemas/record_neighbors}
     record_sensing: {$ref: /schemas/record_sensing}
     use_agent_uid_as_key: bool
     terminate_when_all_idle_or_stuck: bool
     scenario: {$ref: /schemas/scenario}
     name: string
     run_index: int
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
   terminate_when_all_idle_or_stuck: true
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

