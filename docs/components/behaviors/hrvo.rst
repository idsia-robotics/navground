===============================
Hybrid Velocity Obstacle (HRVO)
===============================

   Snape, J., Van Den Berg, J., Guy, S.J. and Manocha, D., 2011. The hybrid reciprocal velocity obstacle. IEEE Transactions on Robotics, 27(4), pp.696-706.

A wrapper of the `open-source implementation <https://github.com/snape/HRVO>`_ (Apache License 2.0) by Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha.

HRVO is another commonly used algorithm from the family of using reciprocal velocity obstacles like :doc:`orca` (read more `from authors <http://gamma.cs.unc.edu/HRVO>`_).

Example
=======

.. video:: hrvo.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video hrvo.yaml hrvo.mp4 --factor 5

with the following configuration

.. literalinclude :: hrvo.yaml
   :language: YAML








