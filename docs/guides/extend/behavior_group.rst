=============== 
Behavior groups
===============

Behaviors may be grouped to compute their commands all together, like for example to implement a centralized controller.


Groups
======

Behaviors group must implement a single interface ``compute_cmds`` (:cpp:func:`C++ <navground::core::BehaviorGroup::compute_cmds>`, :py:meth:`Python <navground.core.BehaviorGroup.compute_cmds>`) that computes the commands for all members. Commands should be kinematically feasible, but it is not strictly required.


Virtual methods
---------------

.. list-table::
   :widths: 45 45 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`compute_cmds <navground::core::BehaviorGroup::compute_cmds>` 
     - :py:meth:`compute_cmds <navground.core.BehaviorGroup.compute_cmds>` 
     - must


Class skeleton
--------------

.. tabs::

   .. tab:: C++

      .. literalinclude :: behavior_group.h
         :language: C++

   .. tab:: Python

      .. literalinclude :: behavior_group.py
         :language: Python

Members
=======

Members of the groups are behaviors that delegates their ``compute_cmd_internal`` (:cpp:func:`C++ <navground::core::Behavior::compute_cmd_internal>`, :py:meth:`Python <navground.core.Behavior.compute_cmd_internal>`) to the group.

Their definition requires to specify:

- how they are grouped using ``get_group_hash`` (:cpp:func:`C++ <navground::core::BehaviorGroupMember::get_group_hash>`, :py:meth:`Python <navground.core.BehaviorGroupMember.get_group_hash>`, behavior with the same key will belong to the same group);

- where groups are stored (globally) using ``get_groups`` (:cpp:func:`C++ <navground::core::BehaviorGroupMember::get_groups>`, :py:meth:`Python <navground.core.BehaviorGroupMember.get_groups>`);

- how to create a new group using ``make_group`` (:cpp:func:`C++ <navground::core::BehaviorGroupMember::make_group>`, :py:meth:`Python <navground.core.BehaviorGroupMember.make_group>`).

.. warning::

   Users should call ``prepare`` (:cpp:func:`C++ <navground::core::Behavior::prepare>`, :py:meth:`Python <navground.core.Behavior.prepare>`) before the first call and
   ``close`` (:cpp:func:`C++ <navground::core::Behavior::close>`, :py:meth:`Python <navground.core.Behavior.close>`) after the final call of a behavior. This will setup and tear down the groups.

Virtual methods
---------------

.. list-table::
   :widths: 45 45 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`get_groups <navground::core::BehaviorGroupMember::get_groups>` 
     - :py:meth:`get_groups <navground.core.BehaviorGroup.get_groups>` 
     - must
   * - :cpp:func:`make_group <navground::core::BehaviorGroupMember::make_group>` 
     - :py:meth:`make_group <navground.core.BehaviorGroup.make_group>` 
     - must
   * - :cpp:func:`get_group_hash <navground::core::BehaviorGroupMember::get_group_hash>` 
     - :py:meth:`get_group_hash <navground.core.BehaviorGroup.get_group_hash>` 
     - can


Class skeleton
--------------

.. tabs::

   .. tab:: C++

      .. literalinclude :: behavior_group_member.h
         :language: C++

   .. tab:: Python

      .. literalinclude :: behavior_group_member.py
         :language: Python
