==========
Kinematics
==========

.. todo:: 

   Should expose the feasible vs 2 too (is it possible?). Maybe better to change names.
   or to fuse both interfaces (passing the current twist ... which may be ignored and set to 0).

   I also don't like that I need to expose ``is_wheeled``.

   And get_max_angular_speed ... i.e., should do some refactoring here.
   
:cpp:func:`navground::core::Kinematics::feasible`
:cpp:func:`navground::core::Kinematics::is_wheeled`
:cpp:func:`navground::core::Kinematics::dof`
:cpp:func:`navground::core::Kinematics::get_max_angular_speed`

.. code-block:: c++

   #include "navground/core/kinematics.h"

   namespace core = navground::core;

   struct MyKinematics : public core::Kinematics {
   
     // must override
     core::Twist2 feasible(const core::Twist2 &value) const override {
       // return the nearest feasible twist to value.
       return core::Twist2(...);
     }
   
     // must override
     bool is_wheeled() const override {
       // return whether we are using wheels
       return ...;
     }

     // must override
     unsigned dof() const override {
       // return the number of dof
       return ...;
     }

     // can override
     float get_max_angular_speed() const override {
       // TODO;
     }

   };
