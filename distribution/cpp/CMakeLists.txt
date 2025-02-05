cmake_minimum_required(VERSION 3.17)

set(CMAKE_OSX_DEPLOYMENT_TARGET 10.15)

project(navground VERSION 0.3.90)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(MSVC)
  add_compile_options(/bigobj)
endif()

set(BUILD_TESTING
    OFF
    CACHE INTERNAL "")

option(STANDALONE "Whether to include all dependencies" ON)
option(CPACK_INSTALL_NAVGROUND_IN_OPT "Whether to install navground in /opt" ON)

if (STANDALONE)
  include(${CMAKE_CURRENT_LIST_DIR}/deps.cmake)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../navground_core
                 ${CMAKE_CURRENT_BINARY_DIR}/navground_core)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../navground_sim
                 ${CMAKE_CURRENT_BINARY_DIR}/navground_sim)

if(STANDALONE)
  if(WIN32)
    install(FILES setup.bat DESTINATION .)
    install(FILES setup.ps1 DESTINATION .)
  elseif(APPLE)
    install(
      FILES setup.mac.zsh
      DESTINATION .
      RENAME setup.zsh)
    install(
      FILES setup.mac.bash
      DESTINATION .
      RENAME setup.bash)
  elseif(UNIX)
    install(
      FILES setup.linux.zsh
      DESTINATION .
      RENAME setup.zsh)
    install(
      FILES setup.linux.bash
      DESTINATION .
      RENAME setup.bash)
  endif()
endif()

set(CPACK_PACKAGE_NAME navground)
set(CPACK_PACKAGE_DESCRIPTION
    "The navground core and simulations libraries")
set(CPACK_PACKAGE_VENDOR "IDSIA")
set(CPACK_VERBATIM_VARIABLES YES)
set(CPACK_PACKAGE_INSTALL_DIRECTORY navground)
set(CPACK_OUTPUT_FILE_PREFIX "packages")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/idsia-robotics/navground")

if(STANDALONE)
  if((NOT WIN32) AND CPACK_INSTALL_NAVGROUND_IN_OPT)
    set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/navground")
  endif()
endif()

if(WIN32)
  set(CPACK_GENERATOR NSIS)
  set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
elseif(APPLE)
  set(CPACK_GENERATOR productbuild)
  set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.rtf")
  set(CPACK_PRODUCTBUILD_IDENTIFIER "org.idsia.navground.cpp")
  set(CPACK_PRODUCTBUILD_DOMAINS true)
  set(CPACK_PRODUCTBUILD_DOMAINS_ANYWHERE true)
  set(CPACK_PRODUCTBUILD_DOMAINS_USER true)
  set(CPACK_PRODUCTBUILD_DOMAINS_ROOT true)
elseif(UNIX)
  set(CPACK_GENERATOR DEB)
  set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
endif()

set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_CONTACT "jerome@idsia.ch")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Jérôme Guzzi")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt")
# set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/../../README.md")

# package name for deb. If set, then instead of some-application-0.9.2-Linux.deb
# you'll get some-application_0.9.2_amd64.deb (note the underscores too)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
# that is if you want every group to have its own package, although the same
# will happen if this is not set (so it defaults to ONE_PER_GROUP) and
# CPACK_DEB_COMPONENT_INSTALL is set to YES
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE) # ONE_PER_GROUP)
# without this you won't be able to pack only specified component
set(CPACK_DEB_COMPONENT_INSTALL OFF)
set(CPACK_STRIP_FILES ON)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
# set(CPACK_MONOLITHIC_INSTALL ON)
if(NOT STANDALONE)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS
      "libargparse-dev, libeigen3-dev, libgeos++-dev, libhdf5-dev, libyaml-cpp-dev"
  )
else()
  # set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6, libgcc-s1, libstdc++6, zlib1g")
endif()

set(CPACK_PACKAGE_FILE_NAME
    ${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}
)

include(CPack)
