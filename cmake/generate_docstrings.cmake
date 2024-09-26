function(generate_docstrings target destination python_src headers)
  add_custom_target(
      docstrings ALL
      COMMAND 
        ${Python3_EXECUTABLE} 
        ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/generate_docstrings.py 
        ${destination} ${python_src} ${headers} ${ARGN} 
      COMMENT "Generate docstrings" 
      OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/docstrings.h
  )
  add_dependencies(${target} docstrings)
endfunction()