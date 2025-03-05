===============
Troubleshooting
===============

HDF5 library release mismatched error 
=====================================

In Windows, if Python cannot import ``navground.sim``

.. code-block:: python

   >>> import navground.sim

   [...]\Lib\site-packages\h5py\__init__.py:36: UserWarning: h5py is running against HDF5 1.14.0 when it was    built against 1.14.2, this may cause problems
     _warn(("h5py is running against HDF5 {0} when it was built against {1}, "
   Warning! ***HDF5 library release mismatched error***
   [...]

set ``HDF5_DISABLE_VERSION_CHECK`` to 1

.. code-block:: console

   $ set HDF5_DISABLE_VERSION_CHECK=1
