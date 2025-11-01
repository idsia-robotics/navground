option(USE_STATIC_LIBS "Whether to link to static libraries only" OFF)

include(FetchContent)

FetchContent_Declare(
  argparse
  GIT_REPOSITORY https://github.com/p-ranav/argparse.git
  GIT_TAG v3.2
  GIT_SHALLOW TRUE
  OVERRIDE_FIND_PACKAGE)
set(ARGPARSE_INSTALL
    ON
    CACHE INTERNAL "")
set(ARGPARSE_BUILD_TESTS
    OFF
    CACHE INTERNAL "")
FetchContent_MakeAvailable(argparse)

FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 5.0.0
  GIT_SHALLOW TRUE
  OVERRIDE_FIND_PACKAGE)
set(EIGEN_BUILD_PKGCONFIG
    OFF
    CACHE INTERNAL "")
set(EIGEN_BUILD_TESTING
    OFF
    CACHE INTERNAL "")
set(EIGEN_BUILD_DOC
    OFF
    CACHE INTERNAL "")
set(EIGEN_BUILD_CMAKE_PACKAGE
    ON
    CACHE INTERNAL "")
FetchContent_MakeAvailable(Eigen3)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
  GIT_SHALLOW TRUE
  OVERRIDE_FIND_PACKAGE)
set(YAML_CPP_INSTALL
    ON
    CACHE INTERNAL "")
if(USE_STATIC_LIBS)
  set(YAML_BUILD_SHARED_LIBS OFF)
else()
  set(YAML_BUILD_SHARED_LIBS ON)
endif()
FetchContent_MakeAvailable(yaml-cpp)

FetchContent_Declare(
  geos
  GIT_REPOSITORY https://github.com/libgeos/geos.git
  GIT_TAG 3.14.1
  GIT_SHALLOW TRUE
  OVERRIDE_FIND_PACKAGE)
set(GEOS_BUILD_DEVELOPER
    OFF
    CACHE INTERNAL "")
set(BUILD_GEOSOP
    OFF
    CACHE INTERNAL "")

if(USE_STATIC_LIBS)
  set(BUILD_SHARED_LIBS OFF)
else()
  set(BUILD_SHARED_LIBS ON)
endif()

FetchContent_MakeAvailable(geos)

set(HDF5_EXTERNALLY_CONFIGURED
    1
    CACHE INTERNAL "")
set(HDF5_EXPORTED_TARGETS
    "navground_simTargets"
    CACHE INTERNAL "")
if(WIN32)
set(HDF5_ENABLE_Z_LIB_SUPPORT
    OFF
    CACHE INTERNAL "")
endif()

FetchContent_Declare(
  hdf5
  GIT_REPOSITORY https://github.com/HDFGroup/hdf5.git
  GIT_TAG hdf5_1.14.6
  GIT_SHALLOW TRUE
  OVERRIDE_FIND_PACKAGE)

set(HDF5_BUILD_TOOLS
    OFF
    CACHE INTERNAL "")
set(HDF5_BUILD_EXAMPLES
    OFF
    CACHE INTERNAL "")
set(HDF5_BUILD_UTILS
    OFF
    CACHE INTERNAL "")

if(USE_STATIC_LIBS)
  set(BUILD_STATIC_LIBS
      ON
      CACHE INTERNAL "")
  set(BUILD_SHARED_LIBS
      OFF
      CACHE INTERNAL "")
else()
  set(BUILD_STATIC_LIBS
      OFF
      CACHE INTERNAL "")
  set(BUILD_SHARED_LIBS
      ON
      CACHE INTERNAL "")
endif()

FetchContent_MakeAvailable(hdf5)

FetchContent_Declare(
  HighFive
  GIT_REPOSITORY https://github.com/highfive-devs/highfive
  GIT_TAG v3.2.0
  GIT_SHALLOW TRUE
  OVERRIDE_FIND_PACKAGE)
set(HIGHFIVE_FIND_HDF5
    OFF
    CACHE INTERNAL "")
set(HIGHFIVE_USE_BOOST
    OFF
    CACHE INTERNAL "")
set(HIGHFIVE_BUILD_DOCS
    OFF
    CACHE INTERNAL "")
set(HIGHFIVE_EXAMPLES
    OFF
    CACHE INTERNAL "")
set(HIGHFIVE_USE_INSTALL_DEPS
    OFF
    CACHE INTERNAL "")

if(USE_STATIC_LIBS)
  set(HIGHFIVE_STATIC_HDF5 ON)
else()
  set(HIGHFIVE_STATIC_HDF5 OFF)
endif()

FetchContent_MakeAvailable(HighFive)
