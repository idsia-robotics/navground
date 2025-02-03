====================
Behavior modulations
====================

Behavior modulation calls virtual method ``pre`` (:cpp:func:`C++ <navground::core::BehaviorModulation::pre>`, :py:meth:`Python <navground.core.BehaviorModulation.pre>`) 
*before* and virtual method ``post`` (:cpp:func:`C++ <navground::core::BehaviorModulation::post>`, :py:meth:`Python <navground.core.BehaviorModulation.post>`) *after* a behavior is evaluated. 

Specializing a new behavior modulation is a simpler alternative to specialize a new behavior, when the specialization should not change the actual methods called to compute a behavior but modulates the parameters (or even state) based on the state and/or the computed command. 

Like for behavior, the command produced by ``post`` should be kinematically feasible, but it is not strictly required.
You should also take into account that modulation are performed as a stack: when behavior ``b`` is associated to a sequence of modulations ``[m_1, m_2, ..., m_n]``, ``b.compute_cmd`` will call them like

.. code-block::

   m_1(b, time_step)
   ... 
   m_n(b, time_step)
   cmd <- b.compute_cmd_internal(...)
   cmd <- m_n(n, time_step, cmd)
   ...
   cmd <- m_1(n, time_step, cmd)

.. note::

    Although not enforced, each other, modulations should reset in ``post`` any change they have performed on the behavior in ``pre``.


Virtual methods
===============

.. list-table::
   :widths: 45 45 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`pre <navground::core::BehaviorModulation::pre>` 
     - :py:meth:`pre <navground.core.BehaviorModulation.pre>` 
     - can
   * - :cpp:func:`post <navground::core::BehaviorModulation::post>` 
     - :py:meth:`post <navground.core.BehaviorModulation.post>` 
     - can


Class skeleton
===============

.. tabs::

   .. tab:: C++

      .. literalinclude :: behavior_modulation.h
         :language: C++

   .. tab:: Python

      .. literalinclude :: behavior_modulation.py
         :language: Python



