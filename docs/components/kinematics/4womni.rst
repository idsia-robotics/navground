==================================
Four-wheeled omnidirectional drive
==================================

The kinematics of an agent with four independently controlled Mecanum wheels.
The kinematics lets the agent move in any direction, with constrains deriving from the maximal speed of its wheels. 

Example
=======

.. video:: 4womni.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video 4womni.yaml 4womni.mp4 --factor 1 --grid 1 --area -3 -1 3 1

with the following configuration

.. literalinclude :: 4womni.yaml
   :language: YAML








