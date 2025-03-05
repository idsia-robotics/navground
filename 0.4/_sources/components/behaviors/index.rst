.. _behaviors:

=========
Behaviors
=========

A *behavior* is the core abstraction of navground. It describes a generic local navigation algorithm that 
takes into account the *local environment* and its current *ego-state* (position, velocity, parameters, ...) to compute a velocity *command* to move towards a *target*. The types of *ego-state*, *command*, and *target* are shared by all behaviors, while the *local environment* type is specific. For instance, navigation algorithms may represent the environment using images (like grid maps) or list of symbolic objects (like neighbors); therefore, we do not try to find a shared *local environment* abstractions.

In simulation, the behavior's *ego-state* is updated from the agent's state at each step, while

- *targets* are updated by :doc:`../tasks/index`
- *local environments* are updated by :doc:`../state_estimations/index`.


------------

.. toctree::
   :maxdepth: 1

   dummy
   orca
   hrvo
   hl
   social_force
   