=====
World
=====


Schema
^^^^^^

.. code-block:: yaml

   $schema: "https://json-schema.org/draft/2020-12/schema"
   $id: /schemas/world
   title: World
   type: object
   properties:
     obstacles: 
       type: array
       items: {$ref: /schemas/disc}
     walls:
       type: array
       items: {$ref: /schemas/line_segments}
     groups: 
       type: array
       items: {$ref: /schemas/agent}
   required: []

Example
^^^^^^^

.. code-block:: yaml

   walls:
     - [[-1.0, -1.0], [-1.0, 1.0]]
   obstacles:
     - position: [2.0, 0.0]
       radius: 0.5
   groups:
     - kinematics:
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

