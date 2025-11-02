==========
Base class
==========

.. py:currentmodule:: navground.core

.. autoclass:: navground.core.Behavior.Heading

   .. autoattribute:: idle

      never turn

   .. autoattribute:: target_point

      turn towards the target position

   .. autoattribute:: target_angle

      turn towards the target orientation

   .. autoattribute:: target_angular_speed

      follow a target angular speed

   .. autoattribute:: velocity

      turn towards the velocity orientation. This is the only behavior available to constrained kinematics.


.. autoclass:: navground.core.Behavior
   :members:
   :inherited-members:
   :exclude-members: __new__, Heading, Mode
