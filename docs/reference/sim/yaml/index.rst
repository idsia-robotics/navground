============
YAML Schemas
============

The YAML formats are specified using `JSON Schema <https://json-schema.org>`_.
You can get the full schema using :ref:`schema_sim` (C++ components only), :ref:`schema_sim_py` (C++ and Python components), like

.. command-output:: navground schema sim
   :ellipsis: 10


To validate a ``<type>`` using this schema, add a reference:

.. code-block:: yaml

   $id: http://navground/sim
   $schema: https://json-schema.org/draft/2020-12/schema
   $ref: <type>
   $defs:
     behavior:
   ...

.. note::

   All schema reported in this section are auto-generated using the Python API, see :py:func:`navground.sim.schema.bundle()`.


.. toctree::
   :maxdepth: 2

   agent
   task
   state_estimation
   world
   samplers
   scenario
   experiment
   
