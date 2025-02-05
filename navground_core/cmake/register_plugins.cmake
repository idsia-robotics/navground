# if NAVGROUND_PLUGINS_IN_AMENT_INDEX: add the entry in the ament_index else,
# add it at "share/navground/plugins"

set(NAVGROUND_PLUGINS_INDEX "share/navground/plugins")

macro(register_navground_plugins)

  cmake_parse_arguments(ARG "" "DESTINATION" "TARGETS" ${ARGN})
  # message("Register navground plugins")

  foreach(target ${ARG_TARGETS})
    list(APPEND plugins ${ARG_DESTINATION}/$<TARGET_FILE_NAME:${target}>)
  endforeach()

  foreach(target ${ARG_TARGETS})
    get_target_property(OUT ${target} LINK_LIBRARIES)
    # message("${target} PLUGIN DEPS ${OUT}")
    if("navground_sim::navground_sim" IN_LIST OUT)
      set(content "#include \"navground/sim/plugin_p.cpp\"")
    else()
      if("navground_core::navground_core" IN_LIST OUT)
        set(content "#include \"navground/core/plugin_p.cpp\"")
      endif()
    endif()
    if(content)
      file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${target}/plugin.cpp ${content})
      set_property(SOURCE ${CMAKE_CURRENT_BINARY_DIR}/${target}/plugin.cpp
                   PROPERTY GENERATED 1)
      target_sources(${target}
                     PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/${target}/plugin.cpp)
    endif()
  endforeach()

  string(REPLACE ";" "\n" content "${plugins}")

  if(plugins)
    if(NAVGROUND_PLUGINS_IN_AMENT_INDEX)
      # message("save ${content} to ament index")
      ament_index_register_resource(navground_plugins CONTENT "${content}")
    else()
      set(temp_file "${CMAKE_CURRENT_BINARY_DIR}/plugins/${PROJECT_NAME}")
      # message("save ${content} to file ${NAVGROUND_PLUGINS_INDEX}")
      file(
        GENERATE
        OUTPUT "${temp_file}"
        CONTENT "${content}")
      install(
        FILES "${temp_file}"
        DESTINATION "${NAVGROUND_PLUGINS_INDEX}"
        COMPONENT plugins)
      # file( GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/plugins.dsv" CONTENT
      # "${DSV}")
      # install(FILES "${CMAKE_CURRENT_BINARY_DIR}/plugins.dsv" DESTINATION
      # share/${PROJECT_NAME}/hook)
      if(ament_cmake_FOUND)
        set("DSV" "prepend-non-duplicate;NAVGROUND_PLUGINS_PREFIX;")
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/plugins.dsv" "${DSV}")
        ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/plugins.dsv")
        # See https://github.com/ament/ament_package/issues/145
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/plugins.sh
             "# Dummy .sh file needed for .dsv file to be sourced.")
        ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/plugins.sh")
      endif()

    endif()

  endif()

endmacro()

macro(cpack_navground_plugins)

  option(CPACK_INSTALL_NAVGROUND_IN_OPT
         "Whether to install navground plugins in /opt" ON)

  set(CPACK_PACKAGE_INSTALL_DIRECTORY navground)
  if(NOT WIN32 AND CPACK_INSTALL_NAVGROUND_IN_OPT)
    set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/navground")
  endif()
  if(WIN32)
    set(CPACK_GENERATOR NSIS)
  elseif(APPLE)
    set(CPACK_GENERATOR productbuild)
    set(CPACK_PRODUCTBUILD_DOMAINS true)
    set(CPACK_PRODUCTBUILD_DOMAINS_ANYWHERE true)
    set(CPACK_PRODUCTBUILD_DOMAINS_USER true)
    set(CPACK_PRODUCTBUILD_DOMAINS_ROOT true)
  elseif(UNIX)
    set(CPACK_GENERATOR DEB)
  endif()
  set(CPACK_OUTPUT_FILE_PREFIX "packages")
  set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
  set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)
  set(CPACK_DEB_COMPONENT_INSTALL YES)
  set(CPACK_STRIP_FILES YES)
  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS YES)
  set(CPACK_PACKAGE_FILE_NAME
      ${PROJECT_NAME}-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}
  )
  include(CPack)
  cpack_add_component(plugins)
endmacro()
