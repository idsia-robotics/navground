cmake_minimum_required(VERSION 3.11)

project(
  navground_coppeliasim
  VERSION 0.0
  LANGUAGES CXX C)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

if(NOT LIBPLUGIN_DIR)
  if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
    set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
  else()
    set(LIBPLUGIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libPlugin)
  endif()
endif()
if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH
  ${LIBPLUGIN_DIR}/cmake
  ${COPPELIASIM_INCLUDE_DIR}/cmake
)

option(NAVGROUND_USES_DOUBLE "Whether to use double instead of float" OFF)

if (NAVGROUND_USES_DOUBLE)
  set(NG_FLOAT_T double)
else()
  set(NG_FLOAT_T float)
endif()

find_package(CoppeliaSim 4 REQUIRED)
find_package(navground_core REQUIRED)
find_package(navground_sim REQUIRED)

if(${COPPELIASIM_VERSION} VERSION_GREATER_EQUAL 4.6)
    set(_PLUGIN_NAME simNavground)
    set(_PLUGIN_NAME_XML simNavground)
    set(_KEYWORD PRIVATE)
else()
    set(_PLUGIN_NAME simExtNavground)
    set(_PLUGIN_NAME_XML Navground)    
endif()  

include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)

execute_process(COMMAND bash -c "sed 's/ng_float_t/${NG_FLOAT_T}/g; s/PLUGIN_NAME/${_PLUGIN_NAME_XML}/'  ${CMAKE_CURRENT_SOURCE_DIR}/lua/callbacks.xml > callbacks.xml" WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_BINARY_DIR}/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/lua/simNavground.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h ESCAPE_QUOTES)

coppeliasim_add_plugin(${_PLUGIN_NAME} SOURCES src/plugin.cpp)

target_link_libraries(${_PLUGIN_NAME} ${_KEYWORD} navground_core::navground_core navground_sim::navground_sim)
