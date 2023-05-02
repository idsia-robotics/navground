========
Scenario
========

Group
-----

Schema
^^^^^^

.. code-block:: yaml

   $schema: "https://json-schema.org/draft/2020-12/schema"
   $id: /schemas/group
   title: Group
   type: object
   properties:
     behavior: {$ref: #/$defs/behavior_sampler}
     kinematics: {$ref: #/$defs/kinematic_sampler}
     task: {$ref: #/$defs/task_sampler}
     state_estimation: {$ref: #/$defs/state_estimation_sampler}
     position: {$ref: /schemas/sampler<vector2>}
     orientation: {$ref: /schemas/sampler<number>}
     radius: {$ref: /schemas/sampler<number>}
     control_period: {$ref: /schemas/sampler<number>} 
     id: {$ref: /schemas/sampler<integer>} 
     type: {$ref: /schemas/sampler<string>} 
     number: number
     # Should be unique across groups
     name: string
   required: [type]
   $defs:
     behavior_sampler: 
       properties:
         type: string
         optimal_speed: {$ref: /schemas/sampler<number>}
         optimal_angular_speed: {$ref: /schemas/sampler<number>}
         rotation_tau: {$ref: /schemas/sampler<number>}
         safety_margin: {$ref: /schemas/sampler<number>}
         horizon: {$ref: /schemas/sampler<number>}
         heading: {$ref: /schemas/sampler<string>}
       required: [type]
       additionalProperties: {}
     kinematic_sampler: 
       properties:
         type: string
         max_speed: {$ref: /schemas/sampler<number>}
         max_angular_speed: {$ref: /schemas/sampler<number>} 
       required: [type]
       additionalProperties: {}
     task_sampler:
       properties:
         type: string
       required: [type]
       additionalProperties: {}
     state_estimation_sampler:
       properties:
         type: string
       required: [type]
       additionalProperties: {}

Example
^^^^^^^

.. code-block:: yaml

   type: my_agent_type
   name: my_group
   number: 4
   kinematics:
     type: Omni
     # implicit constant
     max_speed: 1.0  
   behavior:
     type: HL
     safety_margin: 
       # explicit constant
       sampler: constant
       value: 0.5    
     # implicit sequence
     tau: [0.1, 0.2, 0.2, 0.1]
   state_estimation:
     type: Bounded
     # explicit sequence
     range_of_view: 
       sampler: sequence
       value: [0.5, 1.0, 1.5, 2.0]
   # regular
   radius:
     sampler: regular
     from: 0.1
     to: 0.5
     number: 4
   # grid
   position:
     sampler: regular
     from: [0, 0]
     to: [10, 10]
     number: [2, 2]
   # step
   orientation:
     sampler: regular
     from: 0
     step: 0.1
   control_step:
     # uniform random
     sampler: uniform
     from: 0.1
     to: 0.2

Scenario
--------

Schema
^^^^^^

.. code-block:: yaml

   $schema: "https://json-schema.org/draft/2020-12/schema"
   $id: /schemas/scenario
   title: Scenario
   type: object
   properties:
     type: string
     obstacles: 
       type: array
       items: {$ref: /schemas/disc}
     walls:
       type: array
       items: {$ref: /schemas/line_segments}
     groups: 
       type: array
       items: {$ref: /schemas/group}
   required: [type]
   additionalProperties: {}

Example
^^^^^^^

.. code-block:: yaml

   walls:
     - [[-1.0, -1.0], [-1.0, 1.0]]
   obstacles:
     - 
       position: [2.0, 0.0]
       radius: 0.5
   groups:
     - type: my_type
       number: 2
       kinematics:
         type: Omni
         max_speed: 1.0
       behavior:
         type: Dummy
       radius: 0.1
       control_period: 0.1

