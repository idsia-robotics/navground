========
Sampling
========

Schema
^^^^^^

.. https://json-schema.org/blog/posts/dynamicref-and-generics
.. https://www.w3.org/2019/wot/json-schema#ArraySchema


.. code-block:: yaml

   $schema: "https://json-schema.org/draft/2020-12/schema"
   $id: /schemas/sampler<T>
   title: Sampler<T>
   anyOf:
     - $ref: #/$defs/constant<T>
     - $ref: #/$defs/sequence<T>
     - $ref: #/$defs/regular<T>
     - $ref: #/$defs/regular_step<T>
     - $ref: #/$defs/uniform<T>
     - $ref: #/$defs/normal<T>
   $defs:
     constant<T>:
       oneOf:
          - type: T
          - type: object
            properties:
              sampler: 
                const: constant
              value: T
            required: [sampler, value]
     sequence<T>:
       oneOf:
          - type: array
            items: T
          - type: object
            properties:
              sampler:
                const: sequence
              values: 
                type: array   
                items: T
            wrap: 
              enum: [loop, repeat, terminate]
            required: [sampler, values]
     choice<T>:
       type: object
       properties:
         sampler:
           const: choice
         values: 
           type: array   
           items: T
       required: [sampler, values]
     # limited to T=number, vector2
     $ requires step or [to, number]
     regular<T>:
       type: object
       properties:
         sampler: 
           const: regular
         from: T
         step: T 
         to: T
         number: integer
         wrap: 
           enum: [loop, repeat, terminate]
       required: [sampler, from]
     grid:
       type: object
       properties:
         sampler: 
           const: regular
         from: T
         to: T
         numbers: 
           type: array, 
           items: integer, 
           minItems: 2, 
           maxItems: 2
         wrap: 
           enum: [loop, repeat, terminate]
       required: [sampler, from, to, numbers]
     uniform:
       type: object
       properties:
         sampler: 
           const: uniform
         from: number
         to: number 
       required: [sampler, from, to]   
     normal<T>:
       type: object
       properties:
         sampler: 
           const: normal
         min: number
         max: number
         mean: number
         std_dev: number
       required: [sampler, mean, std_dev]


Examples
^^^^^^^^

Constant (implicit)
~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   1.0  

Constant (explicit)
~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   sampler: constant
   value: 0.5    

Sequence (implicit)
~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   [1.0, 2.0, 2.0, 1.0]  

Sequence (explicit)
~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   sampler: sequence
   values: [1.0, 2.0, 2.0, 1.0]   

Choice
~~~~~~

.. code-block:: yaml

   sampler: choice
   values: [1.0, 2.0, 2.0, 1.0]  

Regular
~~~~~~~

.. code-block:: yaml

   sampler: regular
   from: 0.1
   to: 0.5
   number: 4

Regular (step)
~~~~~~~~~~~~~~

.. code-block:: yaml

   sampler: regular
   from: 0.1
   step: 0.1

Grid
~~~~

.. code-block:: yaml

   sampler: regular
   from: [0, 0]
   to: [1, 1]
   number: [2, 2]

Random uniform
~~~~~~~~~~~~~~

.. code-block:: yaml

   sampler: uniform
   from: 0.1
   to: 0.2

Random normal
~~~~~~~~~~~~~~

.. code-block:: yaml

   sampler: normal
   mean: 0.2
   std_dev: 0.1
   min: 0.0
   max: 1.0





