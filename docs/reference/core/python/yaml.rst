.. _core python yaml:

====
YAML
====

.. py::currentmodule:: navground.core

The Python bindings only exposed part of the public functionality described in `the YAML cpp reference <core cpp yaml>`_.
In particular, you can load registered components and dump them to YAML.

.. py:function:: load_behavior(value: str) -> Behavior | None

   Load a behavior from a YAML string.

   :return:
       The loaded behavior or ``None`` if loading fails.

.. py:function:: load_kinematics(value: str) -> Kinematics  | None

   Load a kinematics from a YAML string.

   :return:
       The loaded kinematics or ``None`` if loading fails.

.. py:function:: load_behavior_modulation(value: str) -> BehaviorModulation | None

   Load a behavior modulation from a YAML string.

   :return:
       The loaded behavior modulation or ``None`` if loading fails.


.. autofunction:: navground.core.dump

