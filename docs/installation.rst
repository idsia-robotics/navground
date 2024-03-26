============
Installation
============

We provide installation instruction for macOS, Linux, and Windows.


Preparation
===========


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

.. 
   warning::

..    On Windows, add the following option

..    
   code-block:: console

..       --cmake-args -T ClangCL

..    to each colcon build commands below. It will use Clang, which is the only compiler we have tested successfully on Windows.


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


All-at-once
===========


.. note::

   In case you prefer using a virtual environment for Python, start by creating and activating it:

   .. tabs::

      .. tab:: macOS
   
         .. code-block:: console
      
            python3 -m venv <path_to_the_venv>
            . <path_to_the_venv>/bin/activate
   
      .. tab:: Linux
   
         .. code-block:: console
      
            python3 -m venv <path_to_the_venv>
            . <path_to_the_venv>/bin/activate
   
   
      .. tab:: Windows
   
         .. code-block:: console
   
            python -m venv <path_to_the_venv>
            <path_to_the_venv>/bin/activate.bat


The following script install everything needed to run navground simulations.
Change ``navground_sim`` to another package name, like ``navground_examples``, to install additional packages.

.. tabs::

   .. tab:: macOS

      .. code-block:: console
   
         mkdir ws
         cd ws
         python3 -m pip install colcon-common-extensions vcstool numpy h5py
         vcs import --input https://raw.githubusercontent.com/idsia-robotics/navground/main/colcon/navground.repos
         export COLCON_DEFAULTS_FILE=src/navground/colcon/defaults.yaml
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to navground_sim

   .. tab:: Linux

      .. code-block:: console
   
         mkdir ws
         cd ws
         python3 -m pip install colcon-common-extensions vcstool numpy h5py
         vcs import --input https://raw.githubusercontent.com/idsia-robotics/navground/main/colcon/navground.repos
         export COLCON_DEFAULTS_FILE=src/navground/colcon/defaults.yaml
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to navground_sim


   .. tab:: Windows

      .. code-block:: console

         mkdir ws
         cd ws
         python -m pip install colcon-common-extensions vcstool numpy h5py
         vcs import --input https://raw.githubusercontent.com/idsia-robotics/navground/main/colcon/navground.repos
         set COLCON_DEFAULTS_FILE=src/navground/colcon/defaults.yaml
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to navground_sim
      
.. note::

   You can modify the build configuration by editing the files in ``src/navground/colcon``, see the `colcon documentation <https://colcon.readthedocs.io/en/released/user/configuration.html#colcon-pkg-files>`_:

   - ``defaults.yaml``
   - ``navground.meta``


   To keep the build up-to-date, run

   .. code-block:: console

      vcs custom --args remote update
      vcs pull src
      
   and then run ``colcon build ...`` again.

Step-by-step instructions
=========================

ROS is not required (except for ROS-specific components, see below) but we do use two build tools from ROS which you can install even without ROS:

- `colcon <https://colcon.readthedocs.io/en/released/>`_ to coordinate the installation from source of different packages [and `vcstool <https://github.com/dirk-thomas/vcstool>`_ to simplify managing source code]

  .. code-block:: console

     python3 -m pip install -U colcon-common-extensions [vcstool]

- `ament_cmake <https://github.com/ament/ament_cmake>`_ to manage resources and integrate better with ROS. If you installed ROS, you will already have it. Else, only on Linux, you can install it from binary

  .. tabs::
  
     .. tab:: macOS
  
        Install from source.
  
     .. tab:: Linux
  
        .. code-block:: console
  
           sudo apt install -y build-essential ament-cmake

     .. tab:: Windows
  
        Install from source.

  or you can build it from source (see below).

Then, create a ``colcon`` workspace and clone ``navground``.

.. code-block:: console

    mkdir -p ws/src
    cd ws
    git clone https://github.com/idsia-robotics/navground.git src/navground

If you need build ``ament_cmake``, clone it and then build it with ``colcon``.

.. code-block:: console

    git clone https://github.com/ament/ament_cmake.git src/ament_cmake
    git clone https://github.com/ament/ament_package src/ament_package 
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --packages-up-to ament_cmake

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


Package
-------

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

   python3 -m pip install -U numpy

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

         sudo apt install -y libclang-dev
         python3 -m pip install clang==14

   .. tab:: Windows

      .. code-block:: console

         clang --version
         python -m pip install clang==<version of the install Clang compiler>


Package
-------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_py

..
   warning::

   On Windows, you need to copy the dll library 

   .. code-block:: console

      copy install\bin\navground_core.dll install\Lib\site-packages\navground\core\navground_core.dll 


.. _Simulation:

Simulation (C++ and Python)
###########################

Dependencies
------------

Depends on `Core C++`_ and `Core Python`_.


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

         The current version installed in Ubuntu `is broken <https://answers.launchpad.net/ubuntu/+source/geos/+question/701657>`_. If you encounter any error, consider installing GEOS from source.

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

h5py
^^^^

To be able to reload a simulation from a saved experiment, install ``h5py``

.. code-block:: console

   python3 -m pip install h5py

websockets [optional]
^^^^^^^^^^^^^^^^^^^^^

To visualize a simulation in real-time from a browser, install ``websockets``

.. code-block:: console

   python3 -m pip install websockets


cairosvg [optional]
^^^^^^^^^^^^^^^^^^^

To render a world to png, pdf or raw images, install ``cairosvg``

.. code-block:: console

   python3 -m pip install cairosvg


moviepy [optional]
^^^^^^^^^^^^^^^^^^

To record a video from a simulation, install ``moviepy``

.. code-block:: console

   python3 -m pip install moviepy


Package
-------

Once all dependencies are installed, compile the package using ``colcon``.

.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_sim

.. 
   warning::

   On Windows, you need to copy the dll library 

   .. code-block:: console

      copy install\bin\navground_sim.dll install\Lib\site-packages\navground\sim\navground_sim.dll 


Examples and demos
##################

Depends on `Core C++`_, `Core Python`_, and `Simulation`_.


.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_examples navground_examples_py navground_demos


ROS
###

Depends on `Core C++`_. You also need to have ROS installed and to source its setup script.

.. code-block:: console

   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_msgs navground_ros


CoppeliaSim
###########

Depends on `Simulation`_. You also need to install `coppeliaSim <https://www.coppeliarobotics.com>`_ (versions 4.3, 4.4, 4.5, 4.6 [latest]).


.. code-block:: console

   export COPPELIASIM_ROOT_DIR=<path to the folder containing the programming subfolder>
   colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select navground_coppeliasim

Usage
=====

To use the installed packages, you will need to source the workspace


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


If you have ROS, you can then launch executables with ``ros2 run ...``:

.. code-block:: console

   ros2 run <name_of_the_package> <name_of_the_executable> 

like, for instance:

.. code-block:: console

   ros2 run navground_core info   

If instead you don't have ROS, directly launch the executables located in ``install``:

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         install/lib/navground_core/info

   .. tab:: Linux

      .. code-block:: console

         install/lib/navground_core/info


   .. tab:: Windows

      .. code-block:: console

         install\Lib\navground_core\info.exe


In the rest of the documentation, we omit ``ros2 run ...`` or the full path prefix and only specify the command to run


.. code-block:: console

   info

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
