====
Path
====

Follow a path defined by a list of points. Use :doc:`waypoints` instead when points are sparse and you want to reach them, not just pass by.

Example
=======

.. video:: path.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video path.yaml path.mp4 --factor 5

with the following configuration

.. literalinclude :: path.yaml
   :language: YAML








