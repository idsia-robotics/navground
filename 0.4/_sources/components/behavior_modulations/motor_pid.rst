====================
Motorized Wheels PID 
====================

Computes the required wheel motor torques to achieve the command, then filters then through PID controllers. Requires that the behavior has a :doc:`../kinematics/2wdiffdyn` kinematics.

Example
=======

.. video:: motor_pid.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video motor_pid.yaml motor_pid.mp4 --factor 5

with the following configuration

.. literalinclude :: motor_pid.yaml
   :language: YAML








