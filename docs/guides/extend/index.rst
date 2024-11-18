=======================
How to extend navground
=======================

The designed way to extend navground is by adding new components, that is new :doc:`../../components/behaviors/index`, :doc:`../../components/behavior_modulations/index`, :doc:`../../components/kinematics/index`, :doc:`../../components/state_estimations/index`, :doc:`../../components/tasks/index`, and :doc:`../../components/scenarios/index`. In fact, navground is designed to become a repository of navigation algorithms (behavior), providing all infrastructure needed to test and compare them. 

In C++ and in Python, each type of component has an base class with methods that sub-classes must/can specialize.
The sub-classes can then be integrated into navground by perfoming few steps described in :doc:`register_cpp` and  :doc:`register_py`, through which the component can be converted to/from YAML
and can be used to perform experiments. 

.. seealso::

   Two additional ways to extend navground using the API (i.e., not exposed through YAML) are:

   :doc:`../probes`
      To record custom data that is not provided by default by experiments, for example to record the internal state of a custom behavior.

   Scenario groups and initializers
      Instead of defining a new scenario, you can add groups or initializers to an existing scenario, for example to position all agents in the scenario along a curve.

------------------------------

.. toctree::
   :maxdepth: 1

   register_cpp
   register_py
   behavior
   behavior_modulation
   kinematics
   state_estimation
   task
   scenario