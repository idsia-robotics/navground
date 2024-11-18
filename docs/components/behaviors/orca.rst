=============================================
Optimal Reciprocal Collision Avoidance (ORCA)
=============================================

   Snape, J., Van Den Berg, J., Guy, S.J. and Manocha, D., 2010, October. Smooth and collision-free navigation for multiple robots under differential-drive constraints. In 2010 IEEE/RSJ international conference on intelligent robots and systems (pp. 4584-4589). IEEE.

A wrapper of the `open-source implementation <https://github.com/snape/RVO2>`_ (Apache License 2.0) by Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha.

ORCA is a commonly used algorithm from the family of using reciprocal velocity obstacles.
It partition the velocity space into safe/unsafe parts (using linear constraints) and then returns the nearest velocity to the target velocity (read more from the `authors <http://gamma.cs.unc.edu/RVO2>`_).

Example
=======

.. video:: orca.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video orca.yaml orca.mp4 --factor 5

with the following configuration

.. literalinclude :: orca.yaml
   :language: YAML








