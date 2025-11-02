====
YAML
====

.. py:currentmodule:: navground.sim

Similar as for :ref:`navground.core <core python_yaml>`,
the Python bindings exposed part of the public functionality described in :ref:`the YAML cpp reference <sim cpp yaml>`.
In particular, for classes 
:py:class:`Task`, 
:py:class:`StateEstimation`, 
:py:class:`Scenario`, 
:py:class:`Experiment`, 
:py:class:`Agent`, 
:py:class:`World`, 
:py:class:`Wall`, 
:py:class:`Obstacle`, 
:py:class:`Sampler`,
you can load objects of class ``T`` using ``T.load(...)`` or ``load_<t>(...)``:

.. autofunction:: load_state_estimation
.. autofunction:: load_sensor
.. autofunction:: load_task
.. autofunction:: load_scenario
.. autofunction:: load_obstacle
.. autofunction:: load_wall
.. autofunction:: load_agent
.. autofunction:: load_world
.. autofunction:: load_experiment
.. autofunction:: load_sampler
.. autofunction:: load_group

You can dump objects of the same classes (and :ref:`navground.core classes <core python_yaml>`) to YAML using ``T.dump()`` or

.. autofunction:: navground.sim.dump

Schema
------

:py:mod:`navground.sim` schemas are more complex than :py:mod:`navground.core` schemas due to the presence of samplers. 

.. py:module:: navground.sim.schema
   :synopsis: Simulation YAML schema

:module: :py:mod:`navground.sim.schema`

.. autofunction:: register

.. autofunction:: bundle

