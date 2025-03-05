========
Odometry
========

A sensor that simulates the self-pose tracking done by integrating odometry measurements.
Updates fields "pose" and "twist" with the estimation computed in a coordinate frame located at the agent's initial pose.

Example
=======

.. video:: odometry.mp4
   :loop:
   :width: 780

The video has been recorded in the :doc:`../../tutorials/sensors` notebook with the following configuration

.. literalinclude :: odometry.yaml
   :language: YAML








