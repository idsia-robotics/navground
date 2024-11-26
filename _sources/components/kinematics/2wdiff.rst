==============================
Two-wheeled differential drive
==============================

The kinematics of a large class of wheeled ground robots that have two independent motors driving the left and right wheel or track trains. Velocities are restricted have the same orientation :math:`\theta` as the robot, i.e.

.. math::

   \vec v \proto \vec e(\theta) 

where :math:`\vec e(\alpha)` is the unit vector in direction :math:`\alpha`. Equivalently, velocities in the relative frame have null lateral components. 

This kinematics works on velocities specified relative to the agent. It constrains the lateral speed to be null, while privileging to preserve angular speed. Constraints on linear and angular speed are coupled and defined by the maximal wheel speed.

.. seealso::

   Read :doc:`../../background/two_wheeled_kinematics` to know more about how feasible velocities are computed.


Example
=======

.. video:: 2wdiff.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video 2wdiff.yaml 2wdiff.mp4 --factor 1 --grid 1 --area -3 -1 3 1

with the following configuration

.. literalinclude :: 2wdiff.yaml
   :language: YAML








