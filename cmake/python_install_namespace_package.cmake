# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Install a Python package (and its recursive subpackages)
#
# :param package_name: the Python package name
# :type package_name: string
# :param PACKAGE_DIR: the path to the Python package directory (default:
#   <package_name> folder relative to the CMAKE_CURRENT_LIST_DIR)
# :type PACKAGE_DIR: string
# :param VERSION: the Python package version (default: package.xml version)
# :param VERSION: string

function(python_install_namespace_package package_name)
  cmake_parse_arguments(
    ARG "SKIP_COMPILE" "TARGET;PACKAGE_DIR;VERSION;NAMESPACE" "DATA" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "python_install_namespace_package() called with unused "
      "arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_PACKAGE_DIR)
    set(ARG_PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/${ARG_NAMESPACE}")
  endif()
  if(NOT IS_ABSOLUTE "${ARG_PACKAGE_DIR}")
    set(ABS_PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/${ARG_PACKAGE_DIR}")
  else()
    set(ABS_PACKAGE_DIR "${ARG_PACKAGE_DIR}")
  endif()
  if(NOT ARG_VERSION)
    set(ARG_VERSION "${${PROJECT_NAME}_VERSION}")
  endif()

  if(NOT ARG_SETUP_CFG)
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/setup.cfg")
      set(ARG_SETUP_CFG "${CMAKE_CURRENT_LIST_DIR}/setup.cfg")
    endif()
  elseif(NOT IS_ABSOLUTE "${ARG_SETUP_CFG}")
    set(ARG_SETUP_CFG "${CMAKE_CURRENT_LIST_DIR}/${ARG_SETUP_CFG}")
  endif()

  set(build_dir "${CMAKE_CURRENT_BINARY_DIR}")

  
  list(TRANSFORM ARG_DATA APPEND "'")
  list(TRANSFORM ARG_DATA PREPEND "'")
  list(JOIN ARG_DATA ", " DATA)

  string(CONFIGURE "\
from setuptools import find_namespace_packages
from setuptools import setup

setup(
    name='${package_name}',
    version='${ARG_VERSION}',
    packages=find_namespace_packages(),
    include_package_data=True,
    package_data={'${package_name}': ['${ARG_TARGET}.*', ${DATA}]}
)
" setup_py_content)

  file(GENERATE
    OUTPUT "${build_dir}/setup.py"
    CONTENT "${setup_py_content}"
  )

  file(REMOVE_RECURSE ${build_dir}/build)

  add_custom_target(
    python_copy
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${ABS_PACKAGE_DIR}" "${build_dir}/${ARG_NAMESPACE}"
  )

  include(FindPython3)

  # message("Will run '${Python3_EXECUTABLE} -m pip install . --prefix ${CMAKE_INSTALL_PREFIX}' in ${CMAKE_CURRENT_BINARY_DIR}")

  add_custom_target(
    pip_install ALL
    COMMAND "${Python3_EXECUTABLE}" -m pip install . --prefix "${CMAKE_INSTALL_PREFIX}"
  )

  add_dependencies(pip_install python_copy "${ARG_TARGET}")

  set(setup_cfg "${CMAKE_CURRENT_LIST_DIR}/setup.cfg")
  if(EXISTS "${setup_cfg}")
    add_custom_target(
      setup_cfg_copy
      COMMAND ${CMAKE_COMMAND} -E copy "${setup_cfg}" "${build_dir}"
    )
    add_dependencies(pip_install setup_cfg_copy)
  endif()

endfunction()