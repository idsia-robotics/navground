function(generate_docstrings target destination python_src headers)
  
  # CMAKE_CURRENT_FUNCTION_LIST_DIR is only available in cmake>=3.17
  set(script "${CMAKE_CURRENT_LIST_DIR}/../cmake/generate_docstrings.py")

  file(GLOB_RECURSE file_headers ${headers}/**.h)

  # message("Python_EXECUTABLE ${Python_EXECUTABLE}")
  add_custom_command(
      COMMAND 
        ${Python_EXECUTABLE} 
        # ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/generate_docstrings.py 
        ${script}
        ${destination} ${python_src} ${headers} ${ARGN} 
      OUTPUT ${destination}
      DEPENDS ${python_src} ${file_headers}
  )

  add_custom_target(
      ${target}_docstrings
      DEPENDS ${destination}
      COMMENT "Generate docstrings" 
  ) 

  add_dependencies(${target} ${target}_docstrings)
endfunction()