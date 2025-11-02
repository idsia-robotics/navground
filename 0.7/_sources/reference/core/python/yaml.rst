.. _core python_yaml:

====
YAML
====

.. py:currentmodule:: navground.core

The Python bindings exposed part of the public functionality described in `the YAML cpp reference <core cpp yaml>`_.
In particular, for classes 
:py:class:`Behavior`,
:py:class:`BehaviorModulation`, 
:py:class:`Kinematics`, 
:py:class:`Disc`, 
:py:class:`LineSegment`, 
:py:class:`Neighbor`
you can load objects of class ``T`` using ``T.load(...)`` or ``load_<t>(...)``:

.. autofunction:: load_behavior
.. autofunction:: load_behavior_modulation
.. autofunction:: load_kinematics
.. autofunction:: load_disc
.. autofunction:: load_line_segment
.. autofunction:: load_neighbor

You can dump objects of the same classes to YAML using ``T.dump()`` or

.. autofunction:: dump


Schema
------

Moreover, we expose methods that return `JSON Schema <https://json-schema.org>`_ for all supported types (``Class.schema``, ``Component.register_schema``), and a function that generate the bundle schema used for validation in :ref:`validate_py`.

.. py:module:: navground.core.schema
   :synopsis: YAML schema


:module: :py:mod:`navground.core.schema`

.. py:type:: Schema
   :canonical: dict[str, typing.Any]

.. py:type:: SchemaModifier
   :canonical: collections.abc.Callable[[Schema], None]

.. autofunction:: register

.. autofunction:: bundle