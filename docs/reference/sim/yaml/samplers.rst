========
Sampling
========

Generic samplers
================

:ref:`C++ samplers <samplers_cpp>` generates samples of a generic types ``T``. To define the corresponding JSON-schema, we follow `<https://json-schema.org/blog/posts/dynamicref-and-generics>`_, using ``$dynamicref`` and ``$dynamicanchor`` to mark the data type ``T`` as generic:


.. code-block:: yaml

   <sampler>:
     $id: <sampler>
     $defs:
       of:
         $dynamicAnchor: T
         not: true
     # uses $dynamicRef: "#T"
     # ...

Other schemas, when referring to the sampler schema, they should define  ``T``

.. code-block:: yaml

   $id: <id>
   $ref: <sampler>
     $defs:
       of:
         $dynamicAnchor: T
         # define schema of T   

like, for example, to sample positive integers:

.. code-block:: yaml

   $id: <id>
   $ref: <sampler>
     $defs:
       of:
         $dynamicAnchor: T
         type: integer
         minimum: 0


All generic samplers share a common parameter :cpp:member:`navground::sim::Sampler::once` that freezes the sampler once the first sample has been drawn.  For example, when in the following scenario

.. code-block:: yaml

   groups:
     - number: 10
       radius: [1.0, 2.0, 3.0]
       once: true

all ten agents will be assigned ``radius: 1.0`` in the first run, ``radius: 2.0`` in the second, and so on. Instead, for ``once: false``, the first agent will get ``radius: 1.0``, the second  ``radius: 2.0``, and so on, in any run.

Constant
--------

.. schema:: navground.sim.schema.bundle()["$defs"]["const"]

Example
~~~~~~~

Constant samplers are specified by a value, like ``0.5``, or by an object, like

.. code-block:: yaml

   sampler: constant
   value: 0.5

.. _sequence:

Sequence
--------

.. schema:: navground.sim.schema.bundle()["$defs"]["sequence"]

Example
~~~~~~~

Sequences are specified by an array, like ``[0.5, 1.0]``, or by an object, like

.. code-block:: yaml

   sampler: sequence
   values: [1.0, 2.0, 2.0, 1.0]   


Choice
------

.. schema:: navground.sim.schema.bundle()["$defs"]["choice"]

Example
~~~~~~~

.. code-block:: yaml

   sampler: choice
   values: [1.0, 2.0, 4.0] 
   probabilities: [0.25, 0.5, 0.25]

Regular
-------

.. schema:: navground.sim.schema.bundle()["$defs"]["regular"]

.. note::

   Restricted to numeric types and 2D vectors

Example
~~~~~~~

.. code-block:: yaml

   sampler: regular
   from: 0.1
   step: 0.1

Grid
----

.. schema:: navground.sim.schema.bundle()["$defs"]["grid"]

.. note::

   Restricted to 2D vectors

Example
~~~~~~~

.. code-block:: yaml

   sampler: grid
   from: [0, 0]
   to: [1, 1]
   number: [2, 2]

Uniform
-------

.. schema:: navground.sim.schema.bundle()["$defs"]["uniform"]

.. note::

   Restricted to numeric types.

Example
~~~~~~~

.. code-block:: yaml

   sampler: uniform
   from: 0.1
   to: 0.2

Normal
------

.. schema:: navground.sim.schema.bundle()["$defs"]["normal"]

.. note::

   Restricted to numeric types.

Example
~~~~~~~

.. code-block:: yaml

   sampler: normal
   mean: 0.2
   std_dev: 0.1
   min: 0.0
   max: 1.0


Normal 2D
----------

.. schema:: navground.sim.schema.bundle()["$defs"]["normal2d"]

.. note::

   Restricted to 2D vector.

Example
~~~~~~~

.. code-block:: yaml

   sampler: normal
   mean: [0.0, 1.0]
   std_dev: [1.0, 4.0]
   angle: 0.7853981634

Vectorized
----------

.. schema:: navground.sim.schema.bundle()["$defs"]["vectorized"]

.. note::

   This schema does not fully specify the sampler for scalar values,
   as it would result in a too complex JSON-schema.

   A part from ``{min|max}_size``, all other fields should 
   represent a valid scalar sampler (for the specific scalar type).


Example
~~~~~~~

.. code-block:: yaml
   # the size sampler
   min_size: 10
   max_size: 20
   # the values iid sampler
   sampler: normal
   mean: 0.0
   std_dev: 1.0
   
.. _samplers_yaml:

Samplers collections
====================

Some generic schema works on any type, others are restricted to a subset of types, like uniform samplers that are restricted to numeric types. Therefore, other schemas do not actually refer to the generic schemas directly, but to the allowed set of schemas depending on the type.

- numbers:

  .. schema:: navground.sim.schema.bundle()['$defs']['number_sampler']

- booleans:

  .. schema:: navground.sim.schema.bundle()['$defs']['boolean_sampler']

- strings:

  .. schema:: navground.sim.schema.bundle()['$defs']['string_sampler']

- 2D vectors:

  .. schema:: navground.sim.schema.bundle()['$defs']['vector2_sampler']

- collections of above scalar types:

  .. schema:: navground.sim.schema.bundle()['$defs']['collection_sampler'] 

Example
-------

For a scenario that has string property "name", the corresponding scheme will contain 

.. code-block:: yaml

   # ...
   properties:
     name:
       $id: name
       $ref: sampler
       $defs:
         of: 
           $dynamicRef: T
           type: string
     #...

and will accept any of the following instances

- .. code-block:: yaml

     # constant works on strings
     name: "apple"

- .. code-block:: yaml

     # sequence works on strings
     name: ["apple", "pear"]

- .. code-block:: yaml

     # choice works on strings
     name: 
       sampler: choice
       values: ["apple", "your name"]

but none of these instances

- .. code-block:: yaml

     # wrong type
     name: 1

- .. code-block:: yaml

     # uniform does not work on strings
     name: 
       sampler: uniform
       from: "apple"
       to: "pear"

