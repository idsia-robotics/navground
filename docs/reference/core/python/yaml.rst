.. _core python yaml:

====
YAML
====

.. py::currentmodule:: navground.core

The Python bindings only exposed part of the public functionality described in `the YAML cpp reference <core cpp yaml>`_.
In particular, you can load registered components and dump them to YAML.

.. autofunction:: navground.core.load_behavior

.. autofunction:: navground.core.load_behavior_modulation

.. autofunction:: navground.core.load_kinematics

.. autofunction:: navground.core.dump

.. note::

   Functions ``load_<component>()`` and methods ``<Component>.load()`` are equivalent, like
   for example :py:func:`navground.core.load_behavior` and :py:meth:`navground.core.Behavior.load`.

   Similar is valid for function :py:func:`navground.core.dump` and methods ``<Component>.dump()``, such as :py:meth:`navground.core.Behavior.dump`.

Schema
------

Moreover, we expose methods that return `JSON Schema <https://json-schema.org>`_ for all supported types (``Class.schema``, ``Component.register_schema``), and a function that generate the bundle schema used for validation in :ref:`validate_py`.

.. py:module:: navground.core.schema
   :synopsis: YAML schema

:module: :py:mod:`navground.core.schema`

.. py:type:: Schema
   :canonical: dict[str, typing.Any]

.. py:type:: SchemaModifier
   :canonical: typing.Callable[[Schema], None]

.. autofunction:: navground.core.schema.register

.. autofunction:: navground.core.schema.bundle