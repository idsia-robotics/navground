============
Installation
============

We provide installation instruction for macOS, Linux, and Windows.

.. note::
   
   You can also `try navground on binder <https://mybinder.org/v2/gh/idsia-robotics/navground.git/HEAD?labpath=tutorials>`_ without installing anything. 

Although not required, we suggest using a Python virtual environment.
In this case, start by creating and activating the virtual environment.

   .. tabs::

      .. tab:: Linux & macOS
   
         .. code-block:: console
      
            python3 -m venv <path_to_the_venv>
            . <path_to_the_venv>/bin/activate
   
      .. tab:: Windows
   
         .. code-block:: console
   
            python -m venv <path_to_the_venv>
            <path_to_the_venv>/bin/activate.bat


Binary installation
===================

You can install the navground via pip.

.. code-block:: console

   pip install navground

.. warning::

   This will install just the Python interface of navground, through which you can use and extend navground from Python but not from C++ (for this, you need to install from source).

Source installation
===================

Preparation
+++++++++++


You will need ``git``, a c++-17 compiler, ``cmake``, and ``Python 3`` with ``pip``. 

Install the appropriate package manager for your system

.. tabs::

   .. tab:: macOS

      We are going to use ``brew``, install it from `homebrew <https://brew.sh>`_.

   .. tab:: Linux

      We are going to use ``apt``.


   .. tab:: Windows

      We are going to use ``choco``, install it from `Chocolatey <https://chocolatey.org/install>`_.


.. note::

   All Windows commands below should be executed in a "Native Tools Command Prompt for VS" with admin privileges.


Install the binary dependencies:

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         brew install cmake git python3

   .. tab:: Linux

      .. code-block:: console

         sudo apt install -y build-essential cmake git python3-dev python3-pip

   .. tab:: Windows

      .. code-block:: console

         choco install -y cmake git python3

Clone this repository in the ``src`` directory

.. tabs::

   .. tab:: Linux & macOS

      .. code-block:: console

         mkdir -p ws/src
         cd ws
         git clone https://github.com/idsia-robotics/navground src/navground

   .. tab:: Windows

      .. code-block:: console

         mkdir ws\src
         cd ws
         git clone https://github.com/idsia-robotics/navground src\navground


All-at-once
+++++++++++

The following script installs everything needed to run navground simulations and some examples. Change the packages specified after ``--packages-up-to`` to install a different set of packages.

.. tabs::

   .. tab:: Linux & macOS

      .. code-block:: console
   
         python3 -m pip install colcon-common-extensions vcstool numpy h5py git+https://github.com/jeguzzi/pybind11_mkdoc@rst
         vcs import --input src/navground/installation/deps.repos
         vcs import --input src/navground/installation/ament.repos
         export COLCON_DEFAULTS_FILE=src/navground/colcon/defaults.yaml
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to navground_examples navground_examples_py navground_examples_yaml

   .. tab:: Windows

      .. code-block:: console

         python -m pip install colcon-common-extensions vcstool numpy h5py git+https://github.com/jeguzzi/pybind11_mkdoc@rst
         vcs import --input src/navground/installation/deps.repos
         vcs import --input src/navground/installation/ament.repos
         set COLCON_DEFAULTS_FILE=src/navground/colcon/defaults.yaml
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to navground_examples navground_examples_py navground_examples_yaml
      
.. note::

   You can modify the build configuration (have a look at the `colcon documentation <https://colcon.readthedocs.io/en/released/user/configuration.html#colcon-pkg-files>`_) by editing the files in ``src/navground/colcon``:

   - ``defaults.yaml``
   - ``navground.meta``


   To keep the build up-to-date, run

   .. code-block:: console

      vcs custom --args remote update
      vcs pull src
      
   and then run ``colcon build ...`` again.

Step-by-step
++++++++++++

ROS is not required (except for ROS-specific components, see below) but we do use build tools from ROS which you can install even without ROS:

- `colcon <https://colcon.readthedocs.io/en/released/>`_ to coordinate the installation from source of different packages [and `vcstool <https://github.com/dirk-thomas/vcstool>`_ to simplify managing source code]

  .. code-block:: console

     pip install -U colcon-common-extensions

- `ament_cmake <https://github.com/ament/ament_cmake>`_ to manage resources and integrate better with ROS. If you installed ROS, you  already have it, else you can build it from source (see below) or, only on Linux, possibly from binary

  .. code-block:: console
  
     sudo apt install -y ament-cmake


- `ament_package <https://github.com/ament/ament_package>`_  is needed by ``ament_cmake``. On Linux, you may install it from binary

  .. code-block:: console
  
     sudo apt install -y python3-ament-package


  else you can build it from source (see below).

- `ament_index_cpp <https://github.com/ament/ament_index>`_  is a c++ library to share resources, like plugins. It is also distributed with ROS 2, else you need to build it from source (see below).

If you need to build the ament packages, clone them and build them with ``colcon``.

.. code-block:: console

    git clone https://github.com/ament/ament_cmake.git src/ament_cmake
    git clone https://github.com/ament/ament_package src/ament_package 
    git clone https://github.com/ament/ament_index src/ament_index
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --packages-up-to ament_cmake ament_index_cpp ament_package

.. note::
   
   The ROS build tools are not required to build or run navgroud.
   Have a look at :doc:`guides/install` to learn what they do and how to install navground without them. 

.. _Core C++:

Core (C++)
##########

Dependencies
------------

The core library depends on ``eigen``, ``yaml-cpp``, and ``argparse``

Eigen 3
^^^^^^^

Installation from source
""""""""""""""""""""""""

.. code-block:: console
 
    git clone https://gitlab.com/libeigen/eigen src/eigen
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select Eigen3

Binary installation
"""""""""""""""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

        brew install eigen

   .. tab:: Linux

      .. code-block:: console

        sudo apt install -y libeigen3-dev

   .. tab:: Windows

      .. code-block:: console

        choco install -y eigen

Yaml-cpp
^^^^^^^^

Installation from source
""""""""""""""""""""""""

.. code-block:: console

   git clone https://github.com/jbeder/yaml-cpp.git src/yaml-cpp
   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DYAML_CPP_INSTALL=ON --packages-select YAML_CPP

Binary installation
"""""""""""""""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         brew install yaml-cpp

   .. tab:: Linux

      .. code-block:: console

         sudo apt install -y libyaml-cpp-dev

   .. tab:: Windows

      Install from source

Argparse
^^^^^^^^

Installation from source
""""""""""""""""""""""""

.. code-block:: console

   git clone  https://github.com/p-ranav/argparse.git src/argparse
   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select argparse


Navground
---------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_core

.. note::

   By default, navground uses ``float`` for floating point numbers. You can switch to ``double`` by specifying the option ``NAVGROUND_USES_DOUBLE``. You do this by adding

   .. code-block:: console

      --cmake-args -DNAVGROUND_USES_DOUBLE=ON

   to the command above.


.. _Core Python:

Core (Python)
#############

Dependencies
------------

Depends on `Core C++`_ library.

NumPy
^^^^^

.. code-block:: console

   pip install -U numpy

pybind11
^^^^^^^^

Installation from source
""""""""""""""""""""""""
 
.. code-block:: console

   git clone https://github.com/pybind/pybind11.git src/pybind11
   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF  --packages-select pybind11

Binary installation
"""""""""""""""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         brew install pybind11

   .. tab:: Linux

      .. code-block:: console

         sudo apt install -y pybind11-dev

   .. tab:: Windows

      Install from source

pybind11_mkdoc [optional]
^^^^^^^^^^^^^^^^^^^^^^^^^

Install ``pybind11_mkdoc`` to import docstrings from C++. It is not necessary but will make the API friendlier to use. 

.. code-block:: console

   pip install git+https://github.com/jeguzzi/pybind11_mkdoc@rst

Navground
---------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_core_py

.. _Simulation C++:

Simulation (C++)
################

Dependencies
------------

Depends on `Core C++`_.


GEOS
^^^^

Installation from source
""""""""""""""""""""""""

.. code-block:: console

   git clone https://github.com/libgeos/geos.git src/geos
   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DGEOS_BUILD_DEVELOPER=OFF --packages-select GEOS

Binary installation
"""""""""""""""""""

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         brew install geos

   .. tab:: Linux

      .. code-block:: console

         sudo apt install -y libgeos++-dev

      .. warning::

         The version installed in Ubuntu `may be broken <https://answers.launchpad.net/ubuntu/+source/geos/+question/701657>`_. If you encounter any error, consider installing GEOS from source.

   .. tab:: Windows

      Install from source


HighFive
^^^^^^^^

You need to first install Hdf5 from source

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

         sudo apt install -y libhdf5-dev

   .. tab:: Windows

      Install from source


Then, install HighFive.

.. code-block:: console

   git clone https://github.com/BlueBrain/HighFive.git src/HighFive
   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DHIGHFIVE_UNIT_TESTS=OFF -DHIGHFIVE_USE_BOOST=OFF -DHIGHFIVE_BUILD_DOCS=OFF --packages-select HighFive

Navground
---------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_sim


.. _Simulation Python:

Simulation (Python)
###################

Dependencies
------------

Depends on `Simulation C++`_ and `Core Python`_.

h5py
^^^^

To be able to reload a simulation from a saved experiment, install ``h5py``

.. code-block:: console

   pip install h5py

multiprocess [optional]
^^^^^^^^^^^^^^^^^^^^^^^

We support `multiprocess <https://pypi.org/project/multiprocess/>`_ as an optional alternative of the `multiprocessing` package contained in the Python standard library

.. code-block:: console

   pip install multiprocess


websockets [optional]
^^^^^^^^^^^^^^^^^^^^^

To visualize a simulation in real-time from a browser, install ``websockets``

.. code-block:: console

   pip install websockets


jinjia [optional]
^^^^^^^^^^^^^^^^^^^

To render a world to svg images, install ``Jinja2``

.. code-block:: console

   pip install Jinja2

cairosvg [optional]
^^^^^^^^^^^^^^^^^^^

To render a world to png, pdf or raw images, install ``cairosvg``

.. code-block:: console

   pip install cairosvg


moviepy [optional]
^^^^^^^^^^^^^^^^^^

To record a video from a simulation, install ``moviepy``

.. code-block:: console

   pip install moviepy


Navground
---------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_sim_py


Examples and demos
##################

Depends on `Core C++`_, `Core Python`_, `Simulation C++`_, `Simulation Python`_.


.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_examples navground_examples_py navground_examples_yaml navground_demos


ROS
###

Depends on `Core C++`_. You also need to have ROS installed and to source its setup script.

.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_msgs navground_ros


CoppeliaSim
###########

Depends on `Simulation C++`_. You also need to install `coppeliaSim <https://www.coppeliarobotics.com>`_ (versions 4.3, 4.4, 4.5, 4.6 [latest]).


.. code-block:: console

   export COPPELIASIM_ROOT_DIR=<path to the folder containing the programming subfolder>
   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_coppeliasim

Usage
=====

If you built navground from source, source the workspace first.

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         . install/setup.zsh

   .. tab:: Linux

      .. code-block:: console
         
         . install/setup.bash


   .. tab:: Windows

      .. code-block:: console
        
         install\setup.bat

Run ``navground_py``, which is available from the ``pip`` installation too.

.. tabs::

   .. tab:: Linux & macOS

      .. code-block:: console

         $ navground_py

         Welcome to the navground!
         
         Usage: navground [--help] [--version] {info,run,sample}
         
         Optional arguments:
           -h, --help     shows help message and exits 
           -v, --version  prints version information and exits 
         
         Subcommands:
           info          Lists registered components.
           run           Runs an experiment.
           sample        Samples a world from a scenario.

   .. tab:: Windows

      .. code-block:: console

         $ navground_py.exe

         Welcome to the navground!
         
         Usage: navground [--help] [--version] {info,run,sample}
         
         Optional arguments:
           -h, --help     shows help message and exits 
           -v, --version  prints version information and exits 
         
         Subcommands:
           info          Lists registered components.
           run           Runs an experiment.
           sample        Samples a world from a scenario.


Troubleshooting
===============

HDF5 library release mismatched error 
#####################################

In Windows, if Python cannot import ``navground.sim``

.. code-block:: python

   >>> import navground.sim

   [...]\Lib\site-packages\h5py\__init__.py:36: UserWarning: h5py is running against HDF5 1.14.0 when it was    built against 1.14.2, this may cause problems
     _warn(("h5py is running against HDF5 {0} when it was built against {1}, "
   Warning! ***HDF5 library release mismatched error***
   [...]

set ``HDF5_DISABLE_VERSION_CHECK`` to 1

.. code-block:: console

   set HDF5_DISABLE_VERSION_CHECK=1
