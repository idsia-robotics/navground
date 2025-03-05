==========
Kinematics
==========

Configure the kinematics sub-class by overriding ``dof`` (:cpp:func:`C++ <navground::core::Kinematics::dof>`, :py:meth:`Python <navground.core.Kinematics.dof>`) and ``is_wheeled`` (:cpp:func:`C++ <navground::core::Kinematics::is_wheeled>`, :py:meth:`Python <navground.core.Kinematics.is_wheeled>`) (for which the base class returns ``false``). You must also override ``feasible`` (:cpp:func:`C++ <navground::core::Kinematics::feasible>`, :py:meth:`Python <navground.core.Kinematics.feasible>`) with the specific logic to map an arbitrary twist to a feasible twist. To add constrains on dynamics/acceleration, you can overrie ``feasible_from_current`` (:cpp:func:`C++ <navground::core::Kinematics::feasible_from_current>`, :py:meth:`Python <navground.core.Kinematics.feasible_from_current>`): the base class implementation ignores the current twist and the time step and calls ``feasible``.

You should override 

- ``get_max_speed`` (:cpp:func:`C++ <navground::core::Kinematics::get_max_speed>`, :py:meth:`Python <navground.core.Kinematics.get_max_speed>`) 
- ``get_max_angular_speed`` (:cpp:func:`C++ <navground::core::Kinematics::get_max_angular_speed>`, :py:meth:`Python <navground.core.Kinematics.get_max_angular_speed>`) 

if the feasible maximal speed and angular speed are different than the base class ``max_speed``, and ``max_angular_speed``.


Virtual methods
===============

.. list-table::
   :widths: 45 45 10
   :header-rows: 1

   * - C++ method
     - Python method
     - override
   * - :cpp:func:`feasible <navground::core::Kinematics::feasible>` 
     - :py:meth:`feasible <navground.core.Kinematics.feasible>` 
     - must
   * - :cpp:func:`feasible_from_current <navground::core::Kinematics::feasible_from_current>` 
     - :py:meth:`feasible_from_current <navground.core.Kinematics.feasible_from_current>` 
     - can
   * - :cpp:func:`dof <navground::core::Kinematics::dof>` 
     - :py:meth:`dof <navground.core.Kinematics.dof>` 
     - must
   * - :cpp:func:`is_wheeled <navground::core::Kinematics::is_wheeled>` 
     - :py:meth:`is_wheeled <navground.core.Kinematics.is_wheeled>` 
     - should
   * - :cpp:func:`get_max_speed <navground::core::Kinematics::get_max_speed>` 
     - :py:meth:`get_max_speed <navground.core.Kinematics.get_max_speed>` 
     - can
   * - :cpp:func:`get_max_angular_speed <navground::core::Kinematics::get_max_angular_speed>` 
     - :py:meth:`get_max_angular_speed <navground.core.Kinematics.get_max_angular_speed>` 
     - can

Class skeleton
===============

.. tabs::

   .. tab:: C++

      .. literalinclude :: kinematics.h
         :language: C++

   .. tab:: Python

      .. literalinclude :: kinematics.py
         :language: Python
