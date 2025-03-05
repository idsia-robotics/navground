.. _core python yaml:

====
YAML
====

The Python bindings only exposed part of the public functionality described in `the YAML cpp reference <core cpp yaml>`_.
In particular, you can load registered components and dump them to YAML.

.. py:function:: load_behavior(value: str) -> Optional[Behavior]

   Load a behavior from a YAML string.

   :return:
       The loaded behavior or ``None`` if loading fails.

.. py:function:: load_kinematics(value: str) -> Optional[Kinematics]

   Load a kinematics from a YAML string.

   :return:
       The loaded kinematics or ``None`` if loading fails.

.. autofunction:: navground.core.dump

