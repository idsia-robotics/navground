================
State estimation
================

Schema
^^^^^^

.. code-block:: yaml

  $schema: "https://json-schema.org/draft/2020-12/schema"
  $id: /schemas/state_estimation
  title: StateEstimation
  type: object
  properties:
    type: string
  required: [type]
  additionalProperties: {}

Example
^^^^^^^

.. code-block:: yaml

   type: Bounded
   range_of_view: 1.0

