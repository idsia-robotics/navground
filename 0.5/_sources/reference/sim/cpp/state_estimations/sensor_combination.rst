===========
Combination
===========

.. note:: Deprecated for navground>=0.5

   Use a sequence of sensors instead.

   For instance, replace
   
   .. code:: c++
   
      navground::sim::Agent agent;
      auto se = std::make_shared<navground::sim::SensorCombination>();
      agent.set_state_estimation(se);
      std::vector<std::shared_ptr<navground::sim::Sensor>>sensors = ...;
      se.sensors.set_sensors(sensors);

   with

   .. code:: c++
   
      navground::sim::Agent agent;
      std::vector<std::shared_ptr<navground::sim::Sensor>>sensors = ...;
      agent.set_state_estimations(sensors); 


.. code-block:: cpp
   
   #include "navground/sim/state_estimations/sensor_combination.h"

.. doxygenstruct:: navground::sim::SensorCombination
   :members:
