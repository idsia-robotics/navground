==============
Github release
==============

Check the `latest release <https://github.com/idsia-robotics/navground/releases>`_ from Github.

C++ library `navground::{core|sim}`
===================================

Download the appropriate installer, which is a ``.deb`` package for (Debian-based) Linux, a ``.pkg`` installer for macOs, or a ``.exe`` installer for Windows. 
Pay attention to download the file for the correct architecture. Installers are named like ``navground_<version>_<os>_<architecture>.{exe|pkg|deb}``.


Run the installer. By default, on macOs and Linux, it will install navground in ``/opt/navground``, while on Windows in ``C:\Program Files\navground``.


Python package ``navground.{core|sim}``
=======================================

After installing the C++ library, from the same release, download the corresponding wheel. Pay attention to download the file for the correct architecture and Python version, which you can infer from the names like ``navground_<version>_<python version>_<os>_<architecture>.whl``.

Then run

.. code-block:: console

   $ pip install <path to the wheel>

Add ``[all]`` to install all optional dependencies:

.. code-block:: console

   $ pip install <path to the wheel>[all]

.. note::

   Install these wheels, not the one from PyPi, if you want that the ``navground`` Python package uses the installed shared libraries. In fact, the wheels from PyPi contain a copy of all required shared libraries: if you import a Python package installed from these wheels, it will load its copy of the shared libraries, not the installed ones. 

   This is particularly important when developing C++ plugins: plugins linked against the installed navground c++ libraries will be discovered by the   package installed from the wheels released on Github but not from PyPi wheels.



