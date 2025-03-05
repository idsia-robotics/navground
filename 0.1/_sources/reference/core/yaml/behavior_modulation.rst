===================
Behavior Modulation
===================

Schema
^^^^^^

.. code-block:: yaml

   $schema: "https://json-schema.org/draft/2020-12/schema"
   $id: /schemas/behavior_modulation
   title: BehaviorModulation
   type: object
   properties:
     type: string
     enabled: boolean
   required: [type]
   additionalProperties: {}

Example
^^^^^^^

.. code-block:: yaml

   type: Relaxation
   enabled: true

