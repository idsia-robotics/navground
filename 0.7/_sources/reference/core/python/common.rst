========
Geometry
========

.. py:currentmodule:: navground.core

Two dimensional
===============

.. py:type:: FloatType
   :module: navground.core
   :canonical: numpy.float32

   Same as type :cpp:type:`ng_float_t`, either :py:type:`numpy.float32` or :py:type:`numpy.float64`.

.. py:type:: Vector2
   :module: navground.core
   :canonical: typing.Annotated[numpy.typing.NDArray[FloatType], '[2, 1]']

   A two dimensional :py:class:`numpy.ndarray` of type :cpp:type:`ng_float_t`.

.. py:type:: Vector2Like
   :module: navground.core
   :canonical: typing.Annotated[numpy.typing.ArrayLike, FloatType, '[2, 1]']

   An object convertible to :py:type:`Vector2`. 

.. note::

   Any pybind11 function that accepts :py:type:`Vector2` does in fact accept the less restrictive type :py:type:`Vector2Like`, which is difficult to encode exactly using the Python type system. For example, property :py:data:`navground.core.Pose2.position` can be set to a tuple of two numbers without any issue:

   .. code-block:: python

      >>> from navground import core
      >>> pose = core.Pose2()
      >>> pose.position = (1, 2)
      array([1., 2.])

   On the contrary, pybind11 functions documented to return :py:type:`Vector2` do return this type.

.. autoenum:: navground.core.Frame

.. autoclass:: Pose2
   :members:
   :undoc-members:

.. autoclass:: Twist2
   :members:
   :undoc-members:

.. autofunction:: orientation_of

.. autofunction:: normalize_angle

.. autofunction:: to_relative

.. autofunction:: to_absolute

.. autofunction:: to_relative_point

.. autofunction:: to_absolute_point

.. autofunction:: unit

.. autofunction:: rotate

.. autofunction:: clamp_norm


