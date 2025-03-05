===============
Human-like (HL)
===============

   Guzzi, J., Giusti, A., Gambardella, L.M., Theraulaz, G. and Di Caro, G.A., 2013, May. Human-friendly robot navigation in dynamic environments. In 2013 IEEE international conference on robotics and automation (pp. 423-430). IEEE.


Our navigation behavior that mimic the model for pedestrains described in 

   Moussaïd, Mehdi, Dirk Helbing, and Guy Theraulaz. "How simple rules determine pedestrian behavior and crowd disasters." Proceedings of the National Academy of Sciences 108, no. 17 (2011): 6884-6888.

Human-like performs a search for the direction in which the agents would come nearest to the target before possible collisions. Then is applies an heuristic to select a safe speed in that direction. By construction, Human-like never retract because for it staying in place is always preferable (i.e., nearest to the target) than move away.

Example
=======

.. video:: hl.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video hl.yaml hl.mp4 --factor 5

with the following configuration

.. literalinclude :: hl.yaml
   :language: YAML








