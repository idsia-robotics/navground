============
Social Force
============

   Helbing, D. and Molnar, P., 1995. Social force model for pedestrian dynamics. Physical review E, 51(5), p.4282.


The social force model is a physics-inspired model that introduces potential fields to model the interaction  with obstacles and neighbors. Our Python implementation follow the original paper.

Example
=======

.. video:: social_force.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video social_force.yaml social_force.mp4 --factor 5

with the following configuration

.. literalinclude :: social_force.yaml
   :language: YAML








