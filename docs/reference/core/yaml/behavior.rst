========
Behavior
========

Schema
^^^^^^

.. code-block:: yaml

   $schema: "https://json-schema.org/draft/2020-12/schema"
   $id: /schemas/behavior
   title: Behavior
   type: object
   properties:
     type: string
     optimal_speed: number
     optimal_angular_speed: number
     rotation_tau: number
     safety_margin: number
     horizon: number
     radius: number
     heading: 
       enum: idle, target_point, target_angle, target_angular_speed, velocity
     social_margin: {$ref: /schemas/social_margin}
   required: [type]
   additionalProperties: {}

Example
^^^^^^^

.. code-block:: yaml

   type: HL
   optimal_speed: 1.2
   safety_margin: 0.2
   horizon: 10
   radius: 0.5
   heading: target_point
   social_margin:
     modulation:
      type: linear
      upper: 1.0
     values:
      1: 0.5
      2: 0.25
     default: 0.125

