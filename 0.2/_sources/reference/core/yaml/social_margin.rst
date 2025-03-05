=============
Social margin
=============

Schema
^^^^^^

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/social_margin
   title: SocialMargin
   type: object
   properties:
     default: number
     values: 
       type: object
       patternProperties: ^\d+$
     modulation: 
       type: object
       properties:
         type:
         enum: ["zero", "constant", "linear", "quadratic", "logistic"]
         upper: number     
   required: [type]

Example
^^^^^^^

.. code-block:: yaml

   modulation:
      type: linear
      upper: 1.0
   values:
      1: 0.5
      2: 0.25
   default: 0.125

