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

   Restricted to numeric types and vectors

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

   Restricted to numeric types and vectors

Example
~~~~~~~

.. code-block:: yaml

   sampler: grid
   from: [0, 0]
   to: [1, 1]
   number: [2, 2]

Random uniform
~~~~~~~~~~~~~~

.. schema:: navground.sim.schema.bundle()["$defs"]["uniform"]

.. note::

   Restricted to numeric types.

Example
~~~~~~~

.. code-block:: yaml

   sampler: uniform
   from: 0.1
   to: 0.2

Random normal
~~~~~~~~~~~~~~

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


.. _samplers_yaml:

Samplers collections
====================

Some generic schema works on any type, others are restricted to a subset of types, like uniform samplers that are restricted to numeric types. Therefore, other schemas do not actually refer to the generic schemas directly, but to the allowed set of schemas depending on the type.

- numbers:

  .. schema:: navground.sim.schema.bundle()['$defs']['number_sampler']

- vectors:

  .. schema:: navground.sim.schema.bundle()['$defs']['vector_sampler']

- other types:

  .. schema:: navground.sim.schema.bundle()['$defs']['sampler']


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

