====
Task
====

Schema
^^^^^^

.. code-block:: yaml

   $schema: "https://json-schema.org/draft/2020-12/schema"
   $id: /schemas/task
   title: Task
   type: object
   properties:
     type: string
   required: [type]
   additionalProperties: {}

Example
^^^^^^^

.. code-block:: yaml

   type: Waypoints
   waypoints: [[0.1, 1.2], [3.4, 4.5], [5.6, 7.8]]
   tolerance: 0.2


