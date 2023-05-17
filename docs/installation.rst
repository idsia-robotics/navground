============
Installation
============

Preparation
===========

On macOS start by installing `homebrew <https://brew.sh>`_ if you don't have it already.

You also need to ``git``, a c++-17 compiler, ``cmake``, and ``Python 3`` with ``pip``. 

.. tabs::

   .. tab:: macOS

      .. code-block:: console

        brew install cmake git python3

   .. tab:: Linux

      .. code-block:: console

        apt install -y build-essential cmake git python3-dev python3-pip


We use `colcon <https://colcon.readthedocs.io/en/released/>`_ to manage installation from source

.. code-block:: console

    python3 -m pip install -U colcon-common-extensions


and `ament_cmake <https://github.com/ament/ament_cmake>`_ to manage resources and integrate better with ROS. 
If you installed ROS, you will already have it. Else, on Linux, install it from binary

..  code-block:: console

    apt install -y build-essential ament-cmake

or build it from source (see below).

In both case, create a ``colcon`` workspace and clone this repository.

.. code-block:: console

    mkdir -p my_ws/src
    cd my_ws
    git clone https://github.com/jeguzzi/navground.git src/navground_ws

If you need build ``ament_cmake``, clone it and then build it with ``colcon``

.. code-block:: console

    git clone https://github.com/ament/ament_cmake.git src/ament_cmake
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ament_cmake


.. note::

    To use the installed packages, you will first need to source the workspace. For examples, in a bash shell, you need to run


    .. code-block:: console

        source install/setup.bash


.. _Core C++:

Core (C++)
==========

Dependencies
------------

The core library depends on ``eigen`` and ``yaml-cpp``.

Eigen 3
^^^^^^^

From source
"""""""""""

.. code-block:: console
 
    git clone https://gitlab.com/libeigen/eigen src/eigen
    colcon --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select Eigen3

Binary
""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

        brew install eigen

   .. tab:: Linux

      .. code-block:: console

        apt install -y libeigen3-dev


Yaml-cpp
^^^^^^^^

From source
"""""""""""

.. code-block:: console

    git clone https://github.com/jbeder/yaml-cpp.git src/yaml-cpp
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DYAML_CPP_INSTALL=ON --packages-select YAML_CPP

Binary
""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

        brew install yaml-cpp

   .. tab:: Linux

      .. code-block:: console

        apt install -y libyaml-cpp-dev


Package
-------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

    colcon --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_core


.. _Core Python:

Core (Python)
=============

Dependencies
------------

Depends on `Core C++`_ library.

NumPy
^^^^^

.. code-block:: console

    python3 -m pip install -U numpy

pybind11
^^^^^^^^

From source
"""""""""""
 
.. code-block:: console

    git clone https://github.com/pybind/pybind11.git src/pybind11
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF  --packages-select pybind11

Binary
""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

        brew install pybind11

   .. tab:: Linux

      .. code-block:: console

        apt install -y pybind11-dev


pybind11_mkdoc [optional]
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: console

    python3 -m pip install git+https://github.com/jeguzzi/pybind11_mkdoc@rst


You also need to install libclang.

.. tabs::

   .. tab:: macOS

      Most probably you already have clang installed.
      Just install the python package of the corresponding version.

      .. code-block:: console

        python3 -m pip install clang==14


   .. tab:: Linux

      Install the python package of the corresponding version.

      .. code-block:: console

        apt install -y libclang-dev
        python3 -m pip install clang==14





Package
-------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

    colcon --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_py


.. _Simulation:

Simulation (C++ and Python)
===========================

Dependencies
------------

Depends on `Core C++`_ and `Core Python`_.


GEOS
^^^^

From source
"""""""""""

.. code-block:: console

    git clone https://github.com/libgeos/geos.git src/geos
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DGEOS_BUILD_DEVELOPER=OFF --packages-select GEOS

Binary
""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

        brew install geos

   .. tab:: Linux

      .. code-block:: console

        apt install -y libgeos++-dev


HighFive
^^^^^^^^

You also first need to install Hdf5 from source

.. code-block:: console

    git clone https://github.com/HDFGroup/hdf5.git src/hdf5
    cd src/hdf5
    git checkout tags/hdf5-1_14_0
    cd ../..
    colcon build --merge-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select HDF5 

or from binary

.. tabs::

   .. tab:: macOS

      .. code-block:: console

        brew install hdf5


   .. tab:: Linux

      .. code-block:: console

        apt install -y libhdf5-dev


Then, install HighFive.

.. code-block:: console

    git clone https://github.com/BlueBrain/HighFive.git src/HighFive
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DHIGHFIVE_UNIT_TESTS=OFF -DHIGHFIVE_USE_BOOST=OFF -DHIGHFIVE_BUILD_DOCS=OFF --packages-select HighFive

Package
-------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

    colcon --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_sim


Examples and demos
==================

Depends on `Core C++`_, `Core Python`_, and `Simulation`_.


.. code-block:: console

    colcon --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_examples navground_examples_py navground_demos


ROS
===

Depends on `Core C++`_. You also need to have ROS installed and to source it's setup script.

.. code-block:: console

    colcon --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_msgs navground_ros


CoppeliaSim
===========

Depends on `Core C++`_. You also need to install `coppeliaSim <https://www.coppeliarobotics.com>`_ (versions 4.3, 4.4, 4.5 [latest]).


.. code-block:: console

    export COPPELIASIM_ROOT_DIR=<path to the folder containing the programming subfolder>
    colcon --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_coppeliasim




