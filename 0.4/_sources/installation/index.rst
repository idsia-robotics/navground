============
Installation
============

We provide installation instruction for macOS, Linux, and Windows.

.. note::
   
   You can also `try navground on binder <https://mybinder.org/v2/gh/idsia-robotics/navground/HEAD?labpath=docs%2Ftutorials>`_ without installing anything. 


There are several ways to build/install navground, either from source or from binary distribution. Building navground from source let you customize and install everything you need, but pre-built may be more suitable (easier, faster) in some cases. The different ways to install/run navground, cover the following cases:

- to extend (and run) navground:

  - only from Python: :doc:`install a pre-built wheel from PyPi<pip>`
  - only from C++: :doc:`install the cpp library released on github<github_release>`
  - from C++ and Python: :doc:`install github release<github_release>`
  - or :doc:`build from source<from_source>`

- to just run navground (e.g., to perform experiments in simulation): :doc:`install a pre-built wheel from PyPi<pip>`
- to run navground in ROS 2: :doc:`build from source<from_source>`
- to use docker: :doc:`build or pull one of the docker images<docker>`

Except when building against a binary installation of ROS 2, 
we suggest installing navground in a Python virtual environment.
In this case, start by creating and activating the virtual environment.

   .. tabs::

      .. tab:: Linux & macOS
   
         .. code-block:: console
      
            python3 -m venv <path_to_the_venv>
            . <path_to_the_venv>/bin/activate
   
      .. tab:: Windows
   
         .. code-block:: console
   
            python -m venv <path_to_the_venv>
            <path_to_the_venv>\bin\activate.bat


.. toctree::
   :maxdepth: 1

   pip
   github_release
   docker
   from_source
   step_by_step
   setup_dev
   troubleshooting

