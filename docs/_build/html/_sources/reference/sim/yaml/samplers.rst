========
Sampling
========

Most fields in complex schemas, like :ref:`scenarios <scenario yaml>`, specify *samplers* of values.
Simplified notations are used for elementary samplers, like constant values

.. code-block:: yaml

   field: 1.0  

of sequences of values

.. code-block:: yaml

   field: [1.0, 2.0, 3.0]

while other samplers need to be specified explictly, like

.. code-block:: yaml

   field: 
     sampler: uniform
     from: 1.0
     to: 2.0


All samplers share a common parameter :cpp:member:`navground::sim::Sampler::once` that freezes the sampler once the first sample has been drawn. Set it to apply the same value to all agents in a group.
For example, in a scenario specified by

.. code-block:: yaml

   groups:
     - number: 10
       radius: [1.0, 2.0, 3.0]
       once: true

all ten agents will be assigned ``radius=1.0`` in the first run, ``radius=2.0`` in the second, and so on.
If instead the scenario is specified by

.. code-block:: yaml

   groups:
     - number: 10
       radius: [1.0, 2.0, 3.0]
       once: false

in any run, the first agent will get ``radius=1.0``, the second  ``radius=2.0``, and so on.



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
              once: bool
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
              once: bool
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
         once: bool
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
         once: bool
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
         once: bool
       required: [sampler, from, to, numbers]
     uniform:
       type: object
       properties:
         sampler: 
           const: uniform
         from: number
         to: number 
         once: bool
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
         once: bool
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





