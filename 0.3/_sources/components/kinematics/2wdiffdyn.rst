==================================================
Two-wheeled differential drive with limited torque
==================================================

A more complex version of :doc:`2wdiff` whose wheels have motors with *limited* torque.
Use this kinematics to avoid that agents motors actuate accelerations that would requires an excessive torque.

This kinematics adds coupled constrains on linear and angular acceleration to the velocity constrains.

.. seealso::

   Read :doc:`../../background/two_wheeled_kinematics` to know more about how feasible velocities are computed.

Example
=======

.. video:: 2wdiffdyn.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video 2wdiffdyn.yaml 2wdiffdyn.mp4 --factor 1 --grid 1 --area -3 -1 3 1

with the following configuration

.. literalinclude :: 2wdiffdyn.yaml
   :language: YAML








