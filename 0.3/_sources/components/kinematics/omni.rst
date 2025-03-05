================
Omni-directional 
================

Restricts velocities to inside the region defined by a maximal speed and a maximal angular speed.
It is the only kinematics that operates on twist specified in absolute coordinates too, as it does not consider the agents' orientation.

Example
=======

.. video:: omni.mp4
	:loop:
	:width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video omni.yaml omni.mp4 --factor 1 --grid 1 --area -2.5 -1 2.5 1 --relative_margin 0 --fps 30 --width 1280 

with the following configuration

.. literalinclude :: omni.yaml
   :language: YAML








