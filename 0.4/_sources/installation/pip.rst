====
PyPi
====

You can install the navground from PyPi via pip.

.. code-block:: console

   pip install navground

Add ``[all]`` to install all optional dependencies.

.. code-block:: console

   pip install navground[all]


.. warning::

   This will install just the Python interface of navground, through which you can use and extend navground from Python but not from C++.

The wheel will also install all shared libraries from dependencies inside the package, without polluting your system.

