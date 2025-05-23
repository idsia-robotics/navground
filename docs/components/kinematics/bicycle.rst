=======
Bicycle
=======

The kinematics of vehicles with steering wheels, like bicycles and cars:

.. math::

   \omega = v \frac{\tan \alpha}{l}

where :math:`l` is the distance between the front and rear wheels and 
:math:`\alpha \in [-\alpha_\max, \alpha_\max]` is the steering angle.

Example
=======

.. video:: bicycle.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video bicycle.yaml bicycle.mp4 --factor 1 --grid 1 --area -3 -1.5 3 1.5

with the following configuration

.. literalinclude :: bicycle.yaml
   :language: YAML








