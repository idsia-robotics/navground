======
Marker
======

A simple sensor that updates the position (fields "x" and "y") of a marker relative to the agent. The orientation of the reference frame can be select between: agent-fixed, world-fixed, or oriented along the agent target direction. 


Example
=======

.. video:: marker.mp4
   :loop:
   :width: 780

The video has been recorded in the :doc:`../../tutorials/sensors` notebook with the following configuration

.. literalinclude :: marker.yaml
   :language: YAML

where the marker position reading is used by an ad-hoc modulation to slow down when near the marker.




