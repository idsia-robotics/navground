==========
Relaxation
==========

Applies an exponential relaxation to the output command :math:`v`

.. math::

   \dot v_f = \frac{v - v_f}{\tau}

and returns :math:`v_f`.

Example
=======

.. video:: relaxation.mp4
  :loop:
  :width: 780

The video has been recorded using

.. code-block:: console

   $ navground_py record_video relaxation.yaml relaxation.mp4 --factor 5

with the following configuration

.. literalinclude :: relaxation.yaml
   :language: YAML








