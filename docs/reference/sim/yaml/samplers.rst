========
Sampling
========

Because in navground scenario are (random) generators of worlds, 
all fields, except the ``type`` field of registered components, in :ref:`scenarios <scenario yaml>` specify typed *samplers* (like a sampler of integer).

Generic samplers
================

Samplers uses generic schema. For instance, :ref:`sequence` defines a sequence of values of any type. Other samplers requires specific types. 

The schema uses ``$dynamicref`` and ``$dynamicanchor`` to keep sampled data type ``T`` generic (see `<https://json-schema.org/blog/posts/dynamicref-and-generics>`_).

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


Constant
~~~~~~~~

.. schema:: navground.sim.schema()["$defs"]["const_sampler"]

Constants are specified by a value, like ``0.5``, or by an object, like

.. code-block:: yaml

   sampler: constant
   value: 0.5    


Sequence
~~~~~~~~

.. schema:: navground.sim.schema()["$defs"]["sequence_sampler"]

Sequences are specified by an array, like ``[0.5, 1.0]``, or by an object, like

.. code-block:: yaml

   sampler: sequence
   values: [1.0, 2.0, 2.0, 1.0]   


Choice
~~~~~~

.. schema:: navground.sim.schema()["$defs"]["choice_sampler"]

For example

.. code-block:: yaml

   sampler: choice
   values: [1.0, 2.0, 2.0, 1.0]  

Regular
~~~~~~~

.. schema:: navground.sim.schema()["$defs"]["regular_sampler"]

Restricted to numeric types and vectors. For example

.. code-block:: yaml

   sampler: regular
   from: 0.1
   step: 0.1

Grid
~~~~

.. schema:: navground.sim.schema()["$defs"]["grid_sampler"]

Restricted to vectors. For example

.. code-block:: yaml

   sampler: grid
   from: [0, 0]
   to: [1, 1]
   number: [2, 2]

Random uniform
~~~~~~~~~~~~~~

.. schema:: navground.sim.schema()["$defs"]["uniform_sampler"]

Restricted to numeric types. For example

.. code-block:: yaml

   sampler: uniform
   from: 0.1
   to: 0.2

Random normal
~~~~~~~~~~~~~~

.. schema:: navground.sim.schema()["$defs"]["normal_sampler"]

Restricted to numeric types. For example

.. code-block:: yaml

   sampler: normal
   mean: 0.2
   std_dev: 0.1
   min: 0.0
   max: 1.0



Typed samplers
==============

In complex schemas, fields are associate to samplers of given types. We define 10 typed sampler schema, one for
each of ``boolean``, ``integer``, ``number``, ``string``, ``vector2`` and their respective array types.

.. schema:: {k: v for k, v in navground.sim.schema()['$defs'].items() if k in ('boolean_sample', 'integer_sampler', 'number_sampler', 'string_sampler', 'vector2_sampler', 'boolean_array_sample', 'integer_array_sampler', 'number_array_sampler', 'string_array_sampler', 'vector2_array_sampler')}

For example, integers can be associated to any generic sampler, while strings only to ``const``, ``sequence`` and ``choice``.

Example
=======

For example, if a scenario has a property "name" of type "string", the corresponding scheme will be 

.. code-block:: yaml

   ...
   properties
     name: string_sampler

which accepts any of these instances

.. code-block:: yaml

   name: "apple"

.. code-block:: yaml

   name: ["apple", "pear"]

.. code-block:: yaml

   name: 
     sampler: choice
     values: ["apple", "your name"]

but none of these instances

.. code-block:: yaml

   name: 1

.. code-block:: yaml

   name: 
     sampler: uniform
     from: "apple"
     to: "pear"

