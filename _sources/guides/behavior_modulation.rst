====================
Behavior modulations
====================


.. todo::

   Explain the relationship with the behavior ... why extend this and not that.
   

:cpp:func:`navground::core::BehaviorModulation::pre`
:cpp:func:`navground::core::BehaviorModulation::post`

.. code-block:: c++

   #include "navground/core/behavior_modulation.h"

   namespace core = navground::core;

   struct MyBehaviorModulation : public core::BehaviorModulation {
   
     // optionally override
     void pre(core::Behavior &behavior, ng_float_t time_step) override {
       // modulate the behavior setting parameters while caching their original
       // value.
     }
   
     // optionally override
     core::Twist2 post(Behavior &behavior, ng_float_t time_step,
                       const Twist2 &cmd) {
       // modulate the cmd and reset parameters to the original values.
       return cmd;
     }
   };

:py:meth:`navground.core.BehaviorModulation.pre`
:py:meth:`navground.core.BehaviorModulation.post`

.. code-block:: python

   from navground import core

   class MyModulation(sim.PyBehaviorModulation):

       # optionally override
       def pre(behavior: core.Behavior, time_step: float) -> None:
           # modulate the behavior setting parameters 
           # while caching their original value.
           ...

       # optionally override
       def post(behavior: core.Behavior, time_step: float, cmd: core.Twist2) -> core.Twist2:
           # modulate the cmd and reset parameters to the original values.
           ...
           return cmd

