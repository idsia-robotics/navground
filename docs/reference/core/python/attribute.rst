.. _attributes python:

==========
Attributes
==========

.. py:currentmodule:: navground.core

Attributes are similar to properties but are dynamic, i.e., they are not specified by the class and can change type. Like properties, values are restricted to the types in :py:type:`Attribute`.
Classes with attributes save/load their attributes from YAML.

.. py:type:: Attribute
   :canonical: bool | int | float | str | Vector2 | list[bool] | list[int] | list[float] | list[str] | list[Vector2]

.. autoclass:: HasAttributes
   :members:
   :undoc-members:
   :exclude-members: __init__

