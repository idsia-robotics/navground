=====
Tasks
=====

.. todo:: INTERNAL

   [x] Let task set the target ... without interacting with the controller

At the begin of a simulation, ``prepare``  (:cpp:func:`C++ <navground::sim::Task::prepare>`, :py:meth:`Python <navground.sim.Task.prepare>`) is called: override it to initialize the task.
The base implementation is empty, therefore it is not required to call it.

During the simulation, ``update``  (:cpp:func:`C++ <navground::sim::Task::update>`, :py:meth:`Python <navground.sim.Task.update>`) is called before a behavior is executed: override it to with higher-level logic specific to your task, which should possibly interacting with the lower-level navigation controller by 

- checking the agent's controller (:cpp:func:`C++ <navground::sim::Agent::get_controller>`, :py:data:`Python <navground.sim.Agent.controller>`) state and possibly triggering some action, or
- direcly setting the agent's behavior (:cpp:func:`C++ <navground::sim::Agent::get_behavior>`, :py:data:`Python <navground.sim.Agent.behavior>`) target

yet, task are free to interact in any other way.
The base implementation is empty, therefore it is not required to call it.

To let the simulation know that it can terminates, you should override ``done`` (:cpp:func:`C++ <navground::sim::Task::done>`, :py:meth:`Python <navground.sim.Task.done>`) returning whether the task has finished. The base implementation returns ``false``, resulting in a never-ending task.

When events meaningful for the task happens, they can be logged using ``log_event`` (:cpp:func:`C++ <navground::sim::Task::log_event>`, :py:meth:`Python <navground.sim.Task.log_event>`), passing a sequence of float as payload. In case you want to log data, you should override ``get_log_size (:cpp:func:`C++ <navground::sim::Task::get_log_size>`, :py:meth:`Python <navground.sim.Task.get_log_size>`), returning the size of the payload, which should not change between events. The base implementation returns ``0``, signalling that it won't log data.

.. warning::
   
   In case you log data of a different size, an exception will be raised.


Virtual methods
===============

.. list-table::
   :widths: 45 45 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`prepare <navground::sim::Task::prepare>` 
     - :py:meth:`prepare <navground.sim.Task.prepare>` 
     - can
   * - :cpp:func:`update <navground::sim::Task::update>` 
     - :py:meth:`update <navground.sim.Task.update>` 
     - should
   * - :cpp:func:`done <navground::sim::Task::done>` 
     - :py:meth:`done <navground.sim.Task.done>` 
     - should
   * - :cpp:func:`get_log_size <navground::sim::Task::get_log_size>` 
     - :py:meth:`get_log_size <navground.sim.Task.get_log_size>` 
     - should

Class skelethon
===============

.. tabs::

   .. tab:: C++

      .. literalinclude :: state_estimation.h
         :language: C++

   .. tab:: Python

      .. literalinclude :: state_estimation.py
         :language: Python