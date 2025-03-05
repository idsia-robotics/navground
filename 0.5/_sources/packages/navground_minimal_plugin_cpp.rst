.. _minimal_cpp:

============================
navground_minimal_plugin_cpp
============================

A packages with the skeleton of a ``C++`` plugin, ready to be built and distributed.

The plugins adds a single behavior that does nothing more than the base class.

To build an installer in ``navground_minimal_plugin_cpp/build/package``:

.. tabs::

   .. tab:: macOS

      .. code-block:: console

         $ source /opt/navground/setup.zsh
         $ mkdir -p navground_minimal_plugin_cpp/build
         $ cd navground_minimal_plugin_cpp/build
         $ cmake -DCMAKE_BUILD_TYPE=Release ..
         $ cmake --build .
         $ cpack

   .. tab:: Linux

      .. code-block:: console

         $ source /opt/navground/setup.bash
         $ mkdir -p navground_minimal_plugin_cpp/build
         $ cd navground_minimal_plugin_cpp.build
         $ cmake -DCMAKE_BUILD_TYPE=Release ..
         $ cmake --build .
         $ cpack

   .. tab:: Windows

      .. code-block:: console

         $ "C:\Program Files\navground\setup.bat"
         $ mkdir navground_minimal_plugin_cpp\build
         $ cd navground_minimal_plugin_cpp\build
         $ cmake.exe ..
         $ cmake.exe --build . --config Release
         $ cpack