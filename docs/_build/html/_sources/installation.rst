============
Installation
============

We provide installation instruction for macOS and Linux. Windows is also supported but not yet documented.
On macOS start by installing `homebrew <https://brew.sh>`_ if you don't have it already.

Preparation
===========

You will need ``git``, a c++-17 compiler, ``cmake``, and ``Python 3`` with ``pip``. 

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         $ brew install cmake git python3

   .. tab:: Linux

      .. code-block:: console

         $ sudo apt install -y build-essential cmake git python3-dev python3-pip


ROS is not required (except for ROS-specific components, see below) but we do use two build tools from ROS which you can install even without ROS:

- `colcon <https://colcon.readthedocs.io/en/released/>`_ to coordinate the installation from source of different packages

  .. code-block:: console

     $ python3 -m pip install -U colcon-common-extensions


- `ament_cmake <https://github.com/ament/ament_cmake>`_ to manage resources and integrate better with ROS. If you installed ROS, you will already have it. Else, only on Linux, you can install it from binary

  .. tabs::
  
     .. tab:: macOS
  
        Install from source.
  
     .. tab:: Linux
  
        .. code-block:: console
  
           $ sudo apt install -y build-essential ament-cmake

  or you can build it from source (see below).

Then, create a ``colcon`` workspace and clone ``navground``.

.. code-block:: console

    $ mkdir -p my_ws/src
    $ cd my_ws
    $ git clone https://github.com/idsia-robotics/navground.git src/navground

If you need build ``ament_cmake``, clone it and then build it with ``colcon``.

.. code-block:: console

    $ git clone https://github.com/ament/ament_cmake.git src/ament_cmake
    $ git clone https://github.com/ament/ament_package src/ament_package 
    $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --packages-up-to ament_cmake


.. warning::

    To use the installed packages, you will need to source the workspace. For examples, in a bash shell, you need to run


    .. code-block:: console

       $ source ./install/setup.bash


.. note::
    If you have ROS, you can launch executables with ``ros2 run ...``:

    .. code-block:: console

       $ ros2 run <name_of_the_package> <name_of_the_executable> 

    like, for instance:

    .. code-block:: console

       $ ros2 run navground_core info   

    If instead you don't have ROS, directly launch the executables from ``install/lib/<name_of_the_package>``.

    .. code-block:: console

       $ ./install/lib/navground_core/info

    In the rest of the documentation, we omit ``ros2 run ...`` or the full path prefix and only specify


    .. code-block:: console

       $ info


.. _Core C++:

Core (C++)
==========

Dependencies
------------

The core library depends on ``eigen`` and ``yaml-cpp``.

Eigen 3
^^^^^^^

Installation from source
""""""""""""""""""""""""

.. code-block:: console
 
    $ git clone https://gitlab.com/libeigen/eigen src/eigen
    $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select Eigen3

Binary installation
"""""""""""""""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

        $ brew install eigen

   .. tab:: Linux

      .. code-block:: console

        $ sudo apt install -y libeigen3-dev


Yaml-cpp
^^^^^^^^

Installation from source
""""""""""""""""""""""""

.. code-block:: console

   $ git clone https://github.com/jbeder/yaml-cpp.git src/yaml-cpp
   $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DYAML_CPP_INSTALL=ON --packages-select YAML_CPP

Binary installation
"""""""""""""""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         $ brew install yaml-cpp

   .. tab:: Linux

      .. code-block:: console

         $ sudo apt install -y libyaml-cpp-dev


Package
-------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

   $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_core


.. _Core Python:

Core (Python)
=============

Dependencies
------------

Depends on `Core C++`_ library.

NumPy
^^^^^

.. code-block:: console

   $ python3 -m pip install -U numpy

pybind11
^^^^^^^^

Installation from source
""""""""""""""""""""""""
 
.. code-block:: console

   $ git clone https://github.com/pybind/pybind11.git src/pybind11
   $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF  --packages-select pybind11

Binary installation
"""""""""""""""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         $ brew install pybind11

   .. tab:: Linux

      .. code-block:: console

         $ sudo apt install -y pybind11-dev


pybind11_mkdoc [optional]
^^^^^^^^^^^^^^^^^^^^^^^^^

Install pybind11_mkdoc to import docstrings from C++. It is not necessary but will make the API friendlier to use. 

.. code-block:: console

   $ python3 -m pip install git+https://github.com/jeguzzi/pybind11_mkdoc@rst


You also need to install libclang.

.. tabs::

   .. tab:: macOS

      Most probably you already have clang installed.
      Just install the python package of the corresponding version.

      .. code-block:: console

         $ python3 -m pip install clang==14


   .. tab:: Linux

      Install the python package of the corresponding version.

      .. code-block:: console

         $ sudo apt install -y libclang-dev
         $ python3 -m pip install clang==14





Package
-------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

   $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_py


.. _Simulation:

Simulation (C++ and Python)
===========================

Dependencies
------------

Depends on `Core C++`_ and `Core Python`_.


GEOS
^^^^

Installation from source
""""""""""""""""""""""""

.. code-block:: console

   $ git clone https://github.com/libgeos/geos.git src/geos
   $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DGEOS_BUILD_DEVELOPER=OFF --packages-select GEOS

Binary installation
"""""""""""""""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         $ brew install geos

   .. tab:: Linux

      .. code-block:: console

         $ sudo apt install -y libgeos++-dev

      .. warning::

         The current version installed in Ubuntu `is broken <https://answers.launchpad.net/ubuntu/+source/geos/+question/701657>`_. If you encounter any error, consider installing GEOS from source.


HighFive
^^^^^^^^

You need to first install Hdf5 from source

.. code-block:: console

   $ git clone https://github.com/HDFGroup/hdf5.git src/hdf5
   $ cd src/hdf5
   $ git checkout tags/hdf5-1_14_0
   $ cd ../..
   $ colcon build --merge-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select HDF5 

or from binary

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         $ brew install hdf5


   .. tab:: Linux

      .. code-block:: console

         $ sudo apt install -y libhdf5-dev


Then, install HighFive.

.. code-block:: console

   $ git clone https://github.com/BlueBrain/HighFive.git src/HighFive
   $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DHIGHFIVE_UNIT_TESTS=OFF -DHIGHFIVE_USE_BOOST=OFF -DHIGHFIVE_BUILD_DOCS=OFF --packages-select HighFive


h5py [optional]
^^^^^^^^^^^^^^^

To be able to reload a simulation from a saved experiment, install ``h5py``

.. code-block:: console

   $ python3 -m pip install h5py

h5py [optional]
^^^^^^^^^^^^^^^

To visualize a simulation in real-time from a browser, install ``websockets``

.. code-block:: console

   $ python3 -m pip install websockets


Package
-------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

   $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_sim


Examples and demos
==================

Depends on `Core C++`_, `Core Python`_, and `Simulation`_.


.. code-block:: console

   $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_examples navground_examples_py navground_demos


ROS
===

Depends on `Core C++`_. You also need to have ROS installed and to source its setup script.

.. code-block:: console

   # colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_msgs navground_ros


CoppeliaSim
===========

Depends on `Simulation`_. You also need to install `coppeliaSim <https://www.coppeliarobotics.com>`_ (versions 4.3, 4.4, 4.5 [latest]).


.. code-block:: console

   $ export COPPELIASIM_ROOT_DIR=<path to the folder containing the programming subfolder>
   $ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_coppeliasim




