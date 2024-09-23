# if NAVGROUND_PLUGINS_IN_AMENT_INDEX: add the entry in the ament_index 
# else, add it at "share/navground/plugins"

set(NAVGROUND_PLUGINS_INDEX "share/navground/plugins")

function(register_navground_plugins)
  
  cmake_parse_arguments(ARG "" "DESTINATION" "TARGETS" ${ARGN})
  message("Register navground plugins ${ARG_TARGETS} installed in ${ARG_DESTINATION}")

  foreach (target ${ARG_TARGETS})
    list(APPEND plugins ${ARG_DESTINATION}/$<TARGET_FILE_NAME:${target}>)
  endforeach () 	

  string (REPLACE ";" "\n" content "${plugins}")

  if(plugins)
    if (NAVGROUND_PLUGINS_IN_AMENT_INDEX)  
      message("save ${content} to ament index")
  	  ament_index_register_resource(navground_plugins CONTENT "${content}")
  	else() 
      set(temp_file "${CMAKE_CURRENT_BINARY_DIR}/plugins/${PROJECT_NAME}")
      message("save ${content} to file ${NAVGROUND_PLUGINS_INDEX}")
      file(GENERATE OUTPUT "${temp_file}" CONTENT "${content}")
      install(
        FILES "${temp_file}"
        DESTINATION "${NAVGROUND_PLUGINS_INDEX}"
      )
    endif() 
  endif()

endfunction()