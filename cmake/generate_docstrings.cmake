function(generate_docstrings target destination python_src headers)
  
  # CMAKE_CURRENT_FUNCTION_LIST_DIR is only available in cmake>=3.17
  set(script "${CMAKE_CURRENT_LIST_DIR}/../cmake/generate_docstrings.py")

  # message("Python3_EXECUTABLE ${Python3_EXECUTABLE}")
  add_custom_target(
      ${target}_docstrings ALL
      COMMAND 
        ${Python3_EXECUTABLE} 
        # ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/generate_docstrings.py 
        ${script}
        ${destination} ${python_src} ${headers} ${ARGN} 
      COMMENT "Generate docstrings" 
      OUTPUT ${destination}
  )
  add_dependencies(${target} ${target}_docstrings)
endfunction()