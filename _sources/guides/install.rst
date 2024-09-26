========================================
How to build without the ROS build tools
========================================

In the installation instruction, we make use of two build tools from the ROS. Here we explain their role and how to build navground without them.
   
Colcon
======

`Colcon <https://colcon.readthedocs.io>`_ is a tool to build a collection of package. It takes care of building them in separate directories to avoid conflicts and in the correct order to avoid missing dependencies. It handles C++ packages using ``CMake`` and Python packages using ``setuptools``. It merges all builds in a single ``install`` directory together with scripts like ``install/setup.bash`` to setup the environment (i.e., `PYTHONPATH`, `PATH`, and so on). 

Once navground is built using colcon, there are no runtime dependencies, except for the need to setup the environment:
the ``install`` directory can be moved where we like and running navground does not require colcon to be installed. 

Using colcon is typical in ROS 2 development. Even without ROS, it facilitates handling collection of packages. We recommend using it to users that want to extend navground, in particular if using C++.
Yet, it is not needed to build navground.

Building navground without colcon
---------------------------------

To build navground without colcon, we just need to call the (native) build system. 

C++ packages
~~~~~~~~~~~~

We can manually compile and install all the packages

.. code-block:: console

   cwd=$PWD
   build=$cwd/build
   CMAKE_INSTALL_PREFIX=$cwd/install
   packages="
   argparse 
   eigen3 
   yaml-cpp 
   geos 
   HighFive 
   hdf5 
   pybind11 
   navground/navground_core 
   navground/navground_sim 
   navground/navground_examples 
   navground/navground_examples_yaml 
   navground/navground_core_py 
   navground/navground_sim_py"
   for package in ${packages}; do
     echo "----------- building $package -----------"
     mkdir -p $build/$package
     cd $build/$package
     cmake $cwd/src/$package -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
     cmake --build . -j10
     cmake --install . --prefix $CMAKE_INSTALL_PREFIX
     cd $cwd
     echo "------------ built $package -------------"
   done

Then, export the plugin and the library path 

.. code-block:: console

   export DYLD_FALLBACK_LIBRARY_PATH=`pwd`/lib
   export NAVGROUND_PLUGINS_PREFIX=`pwd`:$NAVGROUND_PLUGINS_PREFIX


Python packages
~~~~~~~~~~~~~~~

We recommend using a virtual environment. Then, call 

.. code-block:: console

   pip install <path_to_the_python_package> --prefix install

for any Pure Python package you want to install to ``install``, like

.. code-block:: console

   pip install src/navground/navground_examples_py --prefix install

Then, export the Python path

.. code-block:: console

   export PYTHONPATH=`pwd`/install/lib/python3.<XX>/site-packages 


Ament
=====

`Ament <https://github.com/ament>`_ provides packages to simplify the management of collection of packages. For navground, we make use of three ament packages if available:

- `ament_cmake <https://github.com/ament/ament_cmake>`_ is a collection of cmake modules which implement common functionalities like exporting targets; 
- `ament_index_cpp <https://github.com/ament/ament_index>`_ is a c++ library to interact with the ament resource index, which is a way for packages to share resources (like navground plugins);
- `ament_package <https://github.com/ament/ament_package>`_ is a Python package to keep track of installed packages and is needed by `ament_cmake`.

Like for colcon, using ament is expected in ROS 2 but it is not necessary to build navground. In fact, you don't need to change anything to build without ament, just don't install it or
specify the option ``-DWITH_AMENT=OFF`` when building ``navground_core``.

When ament is not used, navground plugins are registered on a ad-hoc index. This is the reason why, when not using colcon, we need to export the index location:

.. code-block:: console
   
   export NAVGROUND_PLUGINS_PREFIX=`pwd`:$NAVGROUND_PLUGINS_PREFIX
