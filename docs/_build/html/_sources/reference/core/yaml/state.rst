=================
Environment State
=================

LineSegment
-----------

Schema
^^^^^^

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/line_segment
   title: LineSegment
   type: object
   properties:
     p1: {$ref: /schemas/vector2}
     p2: {$ref: /schemas/vector2}
   required: [p1, p2]

Example
^^^^^^^

.. code-block:: yaml

   [[1.2, -3.4], [-4.5, 6.7]]

Disc
----

Schema
^^^^^^

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/disc
   title: Disc
   type: object
   properties:
     position: {$ref: /schemas/vector2}
     radius: number
   required: [position, radius]

Example
^^^^^^^

.. code-block:: yaml

   position: [1.2, -3.4]
   radius: 0.5

Neighbor
--------

Schema
^^^^^^

.. code-block:: yaml

   $schema: "https://json-schema.org/draft/2020-12/schema"
   $id: /schemas/neighbor
   title: Neighbor
   type: object
   properties:
     id: integer
     position: {$ref: /schemas/vector2}
     velocity: {$ref: /schemas/vector2}
     radius: number
   required: [id, position, velocity, radius]

Example
^^^^^^^

.. code-block:: yaml

   id: 0
   position: [1.2, -3.4]
   velocity: [0.1, 0.3]
   radius: 0.5
