.. _behavior_modulations:

====================
Behavior Modulations
====================

Behavior modulation execute *before* and *after* a behavior is evaluated. They have full access to the behavior but in most cases should be limited to modifying the behavior parameters (*before*, and resetting them *after*) or output (*after* it is computed). They have two main characteristics:

- they are simpler an alternative to sub-classing a behavior, which is a more complex object.
- they can be shared across behaviors, i.e., the same modulation may apply to different behaviors,

The concrete modulations in the navground core library are all working on the output commands: filtering, clipping, ... . Modulations that acts on the behavior parameters are typically more specific and implemented as extensions.

------------

.. toctree::
   :maxdepth: 1

   limit_acceleration
   limit_twist
   motor_pid
   relaxation
   