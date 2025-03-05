========
Datasets
========

.. autoclass:: navground.sim.Dataset
   :members:
   :exclude-members: __new__

   :py:class:`navground.sim.Dataset` supports the buffer protocol and is convertible to and from a :py:class:`numpy.ndarray`:

   >>> from navground import sim
   >>> import numpy
   >>>
   >>> ds = sim.Dataset(data=numpy.array([1, 2, 3], dtype=numpy.uint8))
   >>> ds
   <Dataset: shape (3,), dtype uint8>
   >>> numpy.asarray(ds)
   array([1, 2, 3], dtype=uint8)



