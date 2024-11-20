============
YAML Schemas
============

The YAML formats are specified using `JSON Schema <https://json-schema.org>`_.
You can get the full schema using :ref:`schema` (C++ components only), :ref:`schema_py` (C++ and Python components), like

.. code-block:: console

   $ schema core

   $id: http://navground/core
   $schema: https://json-schema.org/draft/2020-12/schema
   $defs:
     behavior:
   ...

To validate a ``<type>`` using this schema, add a reference:

.. code-block::

   $id: http://navground/core
   $schema: https://json-schema.org/draft/2020-12/schema
   $ref: <type>
   $defs:
     behavior:
   ...

.. note::

   All schema  reported in this section are auto-generated using the Python API, see :py:func:`navground.core.scheme()`.

.. toctree::
   :maxdepth: 2

   common
   register
   kinematics
   social_margin
   behavior_modulation
   state
   behavior
 
   
