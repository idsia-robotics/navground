=====
Discs
=====

A sensor that updates a sensing environment state with the tabular equivalent to the symbolic geometric state computed by :doc:`bounded`. It updates fields "discs" with a fixed-size table containing the nearest *N* neighbors and obstacles in relative coordinates (providing center, radius, velocity, ... depending on the sensor configuration).

It is mainly used to train Machine-learning behaviors to be compared with the model-based navigation behaviors using a symbolic geometric representation of neighbors and obstacles.


Example
=======

.. video:: discs.mp4
   :loop:
   :width: 780

The video has been recorded using one of the `navground_learning tutorials <https://idsia-robotics.github.io/navground_learning/build/html/tutorials/RL-MA.html>`_
with the following configuration

.. literalinclude :: discs.yaml
   :language: YAML








