=====
World
=====

Schema
^^^^^^

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/wall
   title: Wall
   type: object
   properties:
     line: {$ref: /schemas/line_segment}
     uid: number
   required: [line]

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/obstacle
   title: Wall
   type: object
   properties:
     position: {$ref: /schemas/vector2}
     radius: number  
     uid: number
   required: [position, radius]

.. code-block:: yaml

   $schema: "https://json-schema.org/draft/2020-12/schema"
   $id: /schemas/world
   title: World
   type: object
   properties:
     obstacles: 
       type: array
       items: {$ref: /schemas/obstacle}
     walls:
       type: array
       items: {$ref: /schemas/wall}
     groups: 
       type: array
       items: {$ref: /schemas/agent}
     lattice:
       type: object
       properties:
         x: 
           type: array
           items: numbers
           minItems: 2
           maxItems: 2
         y: 
           type: array
           items: numbers
           minItems: 2
           maxItems: 2
       required: []
     bounding_box:
       type: object
       properties:
         min_x: number
         min_y: number
         max_x: number
         max_y: number
       required: []
   required: []

Example
^^^^^^^

.. code-block:: yaml

   walls:
     - line: [[-1.0, -1.0], [-1.0, 1.0]]
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

