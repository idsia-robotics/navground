.. _behavior_yaml:

========
Behavior
========

Schema
======

.. schema:: navground.core.Behavior.base_schema()

Register
--------

.. schema:: navground.core.Behavior.register_schema()

Example
=======

.. code-block:: yaml

   type: HL
   optimal_speed: 1.2
   safety_margin: 0.2
   horizon: 10
   radius: 0.5
   heading: target_point
   social_margin:
     modulation:
       type: linear
       upper: 1.0
     values:
       1: 0.5
       2: 0.25
     default: 0.125

