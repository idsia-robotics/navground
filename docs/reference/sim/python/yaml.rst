====
YAML
====

Similar as for `the core library <core python yaml>`_,
the Python bindings only exposed part of the public functionality described in :ref:`the YAML cpp reference <sim cpp yaml>`.
In particular, you can load registered components

.. py:function:: load_state_estimation(value: str) -> Optional[StateEstimation]

   Load a state estimation from a YAML string.

   :return:
       The loaded state estimation or ``None`` if loading fails.

.. py:function:: load_task(value: str) -> Optional[Task]

   Load a task from a YAML string.

   :return:
       The loaded task or ``None`` if loading fails.

.. py:function:: load_scenario(value: str) -> Optional[Scenario]

   Load a scenario from a YAML string.

   :return:
       The loaded scenario or ``None`` if loading fails.

in addition to :py:class:`navground.sim.Agent`, :py:class:`navground.sim.World`, and :py:class:`navground.sim.Experiment`:


.. py:function:: load_agent(value: str) -> Optional[Agent]

   Load an agent from a YAML string.

   :return:
       The loaded task or ``None`` if loading fails.

.. py:function:: load_world(value: str) -> Optional[World]

   Load a world from a YAML string.

   :return:
       The loaded world or ``None`` if loading fails.

.. py:function:: load_experiment(value: str) -> Optional[Experiment]

   Load a experiment from a YAML string.

   :return:
       The loaded experiment or ``None`` if loading fails.


You can dump objects of the same classes to YAML: 

.. autofunction:: navground.sim.dump

