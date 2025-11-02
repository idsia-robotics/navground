===========
Combination
===========

.. note:: Deprecated for navground>=0.5

   Use a sequence of sensors instead.

   For instance, replace
   
   .. code:: python
   
      agent = navground.sim.Agent()
      agent.state_estimation = navground.sim.state_estimations.SensorCombination()
      agent.state_estimation.sensors = [...]

   with

   .. code:: python
   
      agent = navground.sim.Agent()
      agent.state_estimations = [...]


.. autoclass:: navground.sim.state_estimations.SensorCombination
   :members:
   :show-inheritance:
