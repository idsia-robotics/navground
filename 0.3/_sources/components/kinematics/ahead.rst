=====
Ahead
=====

A kinematics that restricts the agent to move forwards (i.e., it blocks backwards or sidewards movements).
It mimics how pedestrian prefer to face the direction they are moving to.

This kinematics works on velocities specified relative to the agent.

Example
=======

.. video:: ahead.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video ahead.yaml ahead.mp4 --factor 1 --grid 1 --area -3 -1 3 1

with the following configuration

.. literalinclude :: ahead.yaml
   :language: YAML








