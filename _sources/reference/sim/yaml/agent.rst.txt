=====
Agent
=====

Schema
^^^^^^

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/agent
   properties:
     behavior:
       type: {$ref: /schemas/behavior}
     kinematics:
       type: {$ref: /schemas/kinematics}
     task:
       type: {$ref: /schemas/task}
     state_estimation:
       type: {$ref: /schemas/state_estimation}
     position:
       type: {$ref: /schemas/vector2}
     orientation: 
       type: number
     velocity:
       type: {$ref: /schemas/vector2}
     angular_speed: 
       type: number
     radius: 
       type: number
     control_period: 
       type: number
     id: 
       type: number
     type: 
       type: string
     tags: 
       type: array
       items: string
   required: []

Example
^^^^^^^

.. code-block:: yaml

   kinematics:
     type: Omni
     max_speed: 1.0
   behavior:
     type: Dummy
   task:
     type: Waypoints
     waypoints: [[1.0, 0.0]]
     tolerance: 0.1
   radius: 0.1
   control_period: 0.1
   type: my_agent
   id: 0




