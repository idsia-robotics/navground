======
Docker
======

We provide the following `docker files <https://github.com/idsia-robotics/navground/tree/main/docker>`_


.. list-table::
   :widths: auto

   * - ``Dockerfile.source-deps``
     - Build from source navground and all dependencies in ``ubuntu:latest``
   * - ``Dockerfile.binary-deps``
     - Build from source navground and a minimal number of dependencies in ``ubuntu:latest``
   * - ``Dockerfile.ros``
     - Build from source navground in ``ros:latest``
   * - ``Dockerfile.dev`` 
     - Like ``Dockerfile.source-deps`` but keeping all the packages to build navground again
   * - ``Dockerfile.python``
     - Install the pre-built navground wheel in ``python:3.12-slim``
   * - ``Dockerfile.notebook``
     - Install the pre-built navground wheel in ``python:3.12-slim`` and everything needed to run the jupyter notebooks in `tutorials <https://github.com/idsia-robotics/navground/tree/main/docs/tutorials>`_:
