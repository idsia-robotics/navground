============================
Geometric with limited range
============================

The state estimation updating a symbolic environment state composed of lists of neighbors, obstacles, and lines, as observed a limited range around the agent.


Example
=======

.. video:: bounded.mp4
	:loop:
	:width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video bounded.yaml bounded.mp4 --factor 5 --width 1280 

with the following configuration

.. literalinclude :: bounded.yaml
   :language: YAML








