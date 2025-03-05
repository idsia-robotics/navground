=================
State Estimations
=================

At the begin of a simulation, ``prepare``  (:cpp:func:`C++ <navground::sim::StateEstimation::prepare>`, :py:meth:`Python <navground.sim.StateEstimation.prepare>`) is called: override it to initialize the state estimation.
The base implementation is empty, therefore it is not required to call it.

During the simulation, ``update``  (:cpp:func:`C++ <navground::sim::StateEstimation::update>`, :py:meth:`Python <navground.sim.StateEstimation.update>`) is called before a behavior is executed: override it to update the environment state of the agent's behavior (:cpp:func:`C++ <navground::sim::Agent::get_behavior>`, :py:data:`Python <navground.sim.Agent.behavior>`).

At the end of the simulation, ``close``  (:cpp:func:`C++ <navground::sim::StateEstimation::close>`, :py:meth:`Python <navground.sim.StateEstimation.close>`) is called: override it to clean-up any step performed during ``prepare``.

As the behavior may have any type of environment state, you should check that is support the interface your state estimation requires.

In case your behavior uses an new type of environment state, you will need to define at least one state estimation supporting it.

Another reason to define a new state estimation may be to implement a specific error model or sensing limitation. 

Virtual methods
===============

.. list-table::
   :widths: 45 45 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`prepare <navground::sim::StateEstimation::prepare>` 
     - :py:meth:`prepare <navground.sim.StateEstimation.prepare>` 
     - can
   * - :cpp:func:`update <navground::sim::StateEstimation::update>` 
     - :py:meth:`update <navground.sim.StateEstimation.update>` 
     - can
   * - :cpp:func:`update <navground::sim::StateEstimation::close>` 
     - :py:meth:`update <navground.sim.StateEstimation.close>` 
     - can

Class skeleton
===============

.. tabs::

   .. tab:: C++

      .. literalinclude :: state_estimation.h
         :language: C++

   .. tab:: Python

      .. literalinclude :: state_estimation.py
         :language: Python

======
Sensor
======

Sensors are a class of state estimation that supports ``SensingState`` (:cpp:class:`C++ <navground::core::SensingState>`, :py:class:`Python <navground.core.SensingState>`), which stores data fields ("sensor readings") in homogenous multi-dimensional arrays.

Sub-class sensor to specialize a state estimation working on ``SensingState``. In addition to the methods listed above, you must override ``get_description`` (:cpp:func:`C++ <navground::sim::Sensor::get_description>`, :py:meth:`Python <navground.sim.Sensor.get_description>`) to describe the fields that the sensors is going to write (shape, type of data, limits).


Additional virtual methods
==========================

.. list-table::
   :widths: 45 45 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`get_description <navground::sim::Sensor::get_description>` 
     - :py:meth:`get_description <navground.sim.Sensor.get_description>` 
     - must

Class skeleton
===============

.. tabs::

   .. tab:: C++

      .. literalinclude :: sensor.h
         :language: C++

   .. tab:: Python

      .. literalinclude :: sensor.py
         :language: Python
