===============================
Setup a development environment
===============================

You have installed navground using one of the methods illustrated in the previous sections, and now you want to develop your own extensions/plugins.

Using colcon
============

We recommend to use ``colcon`` and ``ament`` during development. They are not required, but will simplify some of the recurrent tasks, like compiling packages in the right order of dependency. 

Navground built from source
---------------------------

If you have build navground in a colcon workspace, there is not much to do: just create your new packages in the src directory. You may also use a different ``colcon`` workspace. In this case, source the original workspace to  `create an overlay <https://colcon.readthedocs.io/en/released/user/using-multiple-workspaces.html>`_.

Navground installed from a release
----------------------------------

If you have installed navground from a binary release, just source the installation directory (e.g., `` source /opt/navground/setup.bash`` in Linux) before building the with ``colcon``:

1. install ``colcon``

   .. code-block:: console

      $ python -m pip install colcon-common-extensions

2. create a source directory

   .. tabs::

      .. tab:: Linux & macOS
   
         .. code-block:: console
   
            $ mkdir -p ws/src
   
      .. tab:: Windows
   
         .. code-block:: console
   
            $ mkdir ws\src

3. source the navground environment

   .. tabs::

      .. tab:: macOS
   
         .. code-block:: console
   
            $ source /opt/navground/setup.zsh
   
      .. tab:: Linux
   
         .. code-block:: console
   
            $ source /opt/navground/setup.bash
   
      .. tab:: Windows
   
         .. code-block:: console
   
            $ "C:\Program Files\navground\setup.bat"

4. build the workspace

   .. code-block:: console
      
      $ cd ws
      $ colcon build --merge-install

5. If you build navground C++ plugins but don't have ``ament_cmake`` installed, export ``NAVGROUND_PLUGINS_PREFIX``

   .. tabs::
   
      .. tab:: macOS and Linux
   
         .. code-block:: console
   
            $ export NAVGROUND_PLUGINS_PREFIX=`pwd`/install:$NAVGROUND_PLUGINS_PREFIX
   
      .. tab:: Windows
   
         .. code-block:: console
            
            $ set "NAVGROUND_PLUGINS_PREFIX=%cd%\install;%NAVGROUND_PLUGINS_PREFIX%"

Navground installed from PyPi
-----------------------------

If you have installed navground from a PyPi wheel, you are restricted to develop Python plugins. In this case, there is not much you need do except sourcing the virtual environment, if you are using one.

Without colcon
==============

Similar considerations apply when you develop without `colcon` and `ament`. In this case, you need to source the environment where navground is installed.

In case you are developing a C++ plugin, you also need to export the location where you are installing it:

.. tabs::

   .. tab:: macOS and Linux

      .. code-block:: console

         $ export NAVGROUND_PLUGINS_PREFIX=<root directory of the plugin installation>:$NAVGROUND_PLUGINS_PREFIX

   .. tab:: Linux

      .. code-block:: console
         
         $ set "NAVGROUND_PLUGINS_PREFIX=<root directory of the plugin installation>;%NAVGROUND_PLUGINS_PREFIX%"

