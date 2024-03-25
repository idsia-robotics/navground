FROM ubuntu:22.04
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

ENV DEBIAN_FRONTEND="noninteractive" TZ="Europe/Zurich"

# sim -> highfive -> libhdf5 -> libboost

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    ament-cmake \
    libhdf5-dev \
    python3-numpy \
    libclang-dev \
   && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN pip3 install -U colcon-common-extensions clang==14 git+https://github.com/jeguzzi/pybind11_mkdoc@rst h5py

RUN mkdir -p /ws/src && cd /ws/src \
    && git clone https://github.com/libgeos/geos.git \
    && git clone https://github.com/pybind/pybind11.git \
    && git clone https://github.com/BlueBrain/HighFive.git \
    && git clone https://github.com/jbeder/yaml-cpp.git \
    && git clone https://gitlab.com/libeigen/eigen \
    && git clone https://github.com/p-ranav/argparse.git

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select Eigen3

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select argparse

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
       -DPYBIND11_TEST=OFF  --packages-select pybind11

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
       -DYAML_CPP_INSTALL=ON --packages-select YAML_CPP

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
       -DGEOS_BUILD_DEVELOPER=OFF --packages-select GEOS

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
       -DHIGHFIVE_UNIT_TESTS=OFF -DHIGHFIVE_USE_BOOST=OFF -DHIGHFIVE_BUILD_DOCS=OFF --packages-select HighFive


RUN cd /ws/src && git clone https://github.com/idsia-robotics/navground.git

RUN cd /ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select  \
        navground_core \
        navground_py \
        navground_sim \
        navground_examples \
        navground_examples_py \
        navground_demos


