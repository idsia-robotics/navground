# Adapted from https://www.marcusfolkesson.se/blog/git-version-in-cmake/

macro(configure_build_info input output)

  find_package(Git)

  if(GIT_EXECUTABLE)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} describe --tags --dirty
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      OUTPUT_VARIABLE GIT_COMMIT
      RESULT_VARIABLE ERROR_CODE
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} branch --show-current
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      OUTPUT_VARIABLE GIT_BRANCH
      RESULT_VARIABLE ERROR_CODE
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif()

  if(GIT_COMMIT STREQUAL "")
    set(GIT_COMMIT ${CMAKE_PROJECT_VERSION}-unknown)
    message(
      WARNING
        "Failed to determine version from Git tags. Using default version \"${CMAKE_PROJECT_VERSION}\"."
    )
  endif()

  string(TIMESTAMP BUILD_STAMP "%Y-%m-%dT%H:%M:%SZ" UTC)

  # message("GIT_VERSION=\"${GIT_VERSION}\" BUILD_STAMP=\"${BUILD_STAMP}\"")

  # target_compile_definitions(${TARGET} PRIVATE 
  #   GIT_COMMIT="${GIT_COMMIT}"
  #   BUILD_STAMP="${BUILD_STAMP}" )

  configure_file(${input} ${output} @ONLY)

endmacro()
