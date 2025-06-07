========= 
Behaviors 
=========

Environment state
=================

Behaviors must expose their local environment state through ``get_environment_state`` to let other user update it. The returned type should be a sub-class of ``EnvironmentState`` (:cpp:class:`C++ <navground::core::EnvironmentState>`, :py:class:`Python <navground.core.EnvironmentState>`), which is an empty base class.

Therefore, if your behavior needs a new type of environment state, start by defining the new class

.. tabs::

   .. tab:: C++

      .. code-block:: C++

         #include "navground/core/state.h"
   
         struct MyEnvironmentState : public EnvironmentState {
           using EnvironmentState::EnvironmentState;
           // ...
         };

   .. tab:: Python

      from navground import core

      .. code-block:: C++

         class  MyEnvironmentState(core.EnvironmentState):
             
             def __init__(self):
                 super().__init()

Preparation/Termination
=======================

Override ``prepare``  (:cpp:func:`C++ <navground::core::Behavior::prepare>`, :py:meth:`Python <navground.core.Behavior.prepare>`) with any custom logic to initialize the behavior: it should be called before the first command is computed. For instance, specialize this function to setup a centralized group behavior that explicitly coordinates multiple individual behaviors, or to load a resource that requires the behavior being configured (e.g., to load an environment from a file path stored in a behavior property).

Clean-up any step you perform by overriding ``close`` (:cpp:func:`C++ <navground::core::Behavior::prepare>`, :py:meth:`Python <navground.core.Behavior.prepare>`) which should be called once the behavior stop being evaluated.


Compute a command
=================

Behaviors are complex objects that can be specialized in different ways.
The public method ``compute_cmd`` (:cpp:func:`C++ <navground::core::Behavior::compute_cmd>`, :py:meth:`Python <navground.core.Behavior.compute_cmd>`) calls the virtual method ``compute_cmd_internal`` (:cpp:func:`C++ <navground::core::Behavior::compute_cmd_internal>`, :py:meth:`Python <navground.core.Behavior.compute_cmd_internal>`), which in turn calls forwards the request to different methods depending on which targets are currently in use:

- position along a path: ``cmd_twist_along_path``
- pose: ``cmd_twist_towards_pose``
- position: ``cmd_twist_towards_point``
- orientation: ``cmd_twist_towards_orientation``
- velocity: ``cmd_twist_towards_velocity``
- angular speed: ``cmd_twist_towards_angular_speed``
- none: ``cmd_twist_towards_stopping``.


To specialize a behavior, users may override ``compute_cmd_internal`` or
any of the methods listed above. 

They may also override the following internal methods:

- ``desired_velocity_towards_point``, used by the base ``cmd_twist_towards_point``
- ``desired_velocity_towards_velocity``, used by the base ``cmd_twist_towards_velocity``
- ``twist_towards_velocity``, use by by the base ``cmd_twist_towards_point`` and ``cmd_twist_towards_velocity``.

The command produced by ``compute_cmd_internal`` should be kinematically feasible, but it is not strictly required.


Virtual methods
===============

.. list-table::
   :widths: 45 45 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`get_environment_state <navground::core::Behavior::get_environment_state>` 
     - :py:meth:`get_environment_state <navground.core.Behavior.get_environment_state>` 
     - must
   * - :cpp:func:`prepare <navground::core::Behavior::prepare>` 
     - :py:meth:`prepare <navground.core.Behavior.prepare>` 
     - can
   * - :cpp:func:`close <navground::core::Behavior::close>` 
     - :py:meth:`close <navground.core.Behavior.close>` 
     - can
   * - :cpp:func:`compute_cmd_internal <navground::core::Behavior::compute_cmd_internal>` 
     - :py:meth:`compute_cmd_internal <navground.core.Behavior.compute_cmd_internal>` 
     - can
   * - :cpp:func:`cmd_twist_along_path <navground::core::Behavior::cmd_twist_along_path>` 
     - :py:meth:`cmd_twist_along_path <navground.core.Behavior.cmd_twist_along_path>` 
     - can
   * - :cpp:func:`cmd_twist_towards_pose <navground::core::Behavior::cmd_twist_towards_pose>`
     - :py:meth:`cmd_twist_towards_pose <navground.core.Behavior.cmd_twist_towards_pose>` 
     - can
   * - :cpp:func:`cmd_twist_towards_point <navground::core::Behavior::cmd_twist_towards_point>` 
     - :py:meth:`cmd_twist_towards_point <navground.core.Behavior.cmd_twist_towards_point>` 
     - can
   * - :cpp:func:`cmd_twist_towards_velocity <navground::core::Behavior::cmd_twist_towards_velocity>` 
     - :py:meth:`cmd_twist_towards_velocity <navground.core.Behavior.cmd_twist_towards_velocity>` 
     - can
   * - :cpp:func:`cmd_twist_towards_orientation <navground::core::Behavior::cmd_twist_towards_orientation>` 
     - :py:meth:`cmd_twist_towards_orientation <navground.core.Behavior.cmd_twist_towards_orientation>` 
     - can
   * - :cpp:func:`cmd_twist_towards_stopping <navground::core::Behavior::cmd_twist_towards_stopping>`
     - :py:meth:`cmd_twist_towards_stopping <navground.core.Behavior.cmd_twist_towards_stopping>` 
     - can
   * - :cpp:func:`desired_velocity_towards_point <navground::core::Behavior::desired_velocity_towards_point>`
     - :py:meth:`desired_velocity_towards_point <navground.core.Behavior.desired_velocity_towards_point>` 
     - can
   * - :cpp:func:`desired_velocity_towards_velocity <navground::core::Behavior::desired_velocity_towards_velocity>`
     - :py:meth:`desired_velocity_towards_velocity <navground.core.Behavior.desired_velocity_towards_velocity>` 
     - can
   * - :cpp:func:`twist_towards_velocity <navground::core::Behavior::twist_towards_velocity>`
     - :py:meth:`twist_towards_velocity <navground.core.Behavior.twist_towards_velocity>` 
     - can


Class skeleton
===============

.. tabs::

   .. tab:: C++

      .. literalinclude :: behavior.h
         :language: C++

   .. tab:: Python

      .. literalinclude :: behavior.py
         :language: Python


