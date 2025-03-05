==========
Kinematics
==========

Schema
^^^^^^

.. code-block:: yaml

  $schema: "https://json-schema.org/draft/2020-12/schema"
  $id: /schemas/kinematics
  title: Kinematics
  type: object
  properties:
    type: string
    max_speed: number
    max_angular_speed: number
  required: [type]
  additionalProperties: {}

Example
^^^^^^^

.. code-block:: yaml

  type: 2WDiff
  max_speed: 1.5
  wheel_axis: 0.25 

