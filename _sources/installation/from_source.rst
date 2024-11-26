====================
Building from source
====================

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


Install the binary dependencies:

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         $ brew install cmake git python3

   .. tab:: Linux

      .. code-block:: console

         $ sudo apt install -y build-essential cmake git python3-dev python3-pip

   .. tab:: Windows

      .. code-block:: console

         $ choco install -y cmake git python3

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


Building
========

The following script installs everything needed to run navground simulations and some examples. Change the packages specified after ``--packages-up-to`` to install a different set of packages.

.. tabs::

   .. tab:: Linux & macOS

      .. code-block:: console
   
         python3 -m pip install colcon-common-extensions vcstool numpy h5py PyYAML git+https://github.com/jeguzzi/pybind11_mkdoc@rst
         vcs import --shallow --input src/navground/installation/deps.repos
         vcs import --shallow --input src/navground/installation/ament.repos
         export COLCON_DEFAULTS_FILE=src/navground/colcon/defaults.yaml
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to ament_cmake ament_package ament_index_cpp
         source install/setup.bash
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to navground_examples navground_examples_py navground_examples_yaml

   .. tab:: Windows

      .. code-block:: console

         python -m pip install colcon-common-extensions vcstool numpy h5py git+https://github.com/jeguzzi/pybind11_mkdoc@rst
         vcs import --shallow --input src/navground/installation/deps.repos
         vcs import --shallow --input src/navground/installation/ament.repos
         set COLCON_DEFAULTS_FILE=src/navground/colcon/defaults.yaml
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to ament_cmake ament_package ament_index_cpp
         install\setup.bat
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to navground_examples navground_examples_py navground_examples_yaml
      
.. note::

   You can modify the build configuration (have a look at the `colcon documentation <https://colcon.readthedocs.io/en/released/user/configuration.html#colcon-pkg-files>`_) by editing the files in ``src/navground/colcon``:

   - ``defaults.yaml``
   - ``navground.meta``


ROS 2
=====

If you `have installed ROS 2 <https://docs.ros.org/en/jazzy/Installation.html>`_, you will need to build fewer dependencies, you can add ``navground_ros`` to the list of packages, and source the ROS 2 workspace.

.. tabs::

   .. tab:: Linux & macOS

      .. code-block:: console
   
         python3 -m pip install h5py git+https://github.com/jeguzzi/pybind11_mkdoc@rst
         vcs import --input src/navground/installation/deps-ros.repos
         export COLCON_DEFAULTS_FILE=src/navground/colcon/defaults.yaml
         source /opt/ros/<ROS_VERSION>/setup.bash
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to navground_ros navground_examples navground_examples_py navground_examples_yaml

   .. tab:: Windows

      .. code-block:: console

         python -m pip install h5py git+https://github.com/jeguzzi/pybind11_mkdoc@rst
         vcs import --input src/navground/installation/deps-ros.repos
         \dev\ros2_<ROS_VERSION>\local_setup.bat
         set COLCON_DEFAULTS_FILE=src/navground/colcon/defaults.yaml
         colcon build --metas src/navground/colcon/navground.meta --packages-up-to navground_ros navground_examples navground_examples_py navground_examples_yaml

Update
======

To keep the build up-to-date, run

.. code-block:: console

   vcs custom --args remote update
   vcs pull src
   
and then run ``colcon build ...`` again.
