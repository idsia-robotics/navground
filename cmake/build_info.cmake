# Adapted from https://www.marcusfolkesson.se/blog/git-version-in-cmake/

macro(add_build_info TARGET)

  find_package(Git)

  if(GIT_EXECUTABLE)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} describe --tags --dirty
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      OUTPUT_VARIABLE GIT_VERSION
      RESULT_VARIABLE ERROR_CODE
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif()

  if(GIT_VERSION STREQUAL "")
    set(GIT_VERSION 0.0.0-unknown)
    message(
      WARNING
        "Failed to determine version from Git tags. Using default version \"${GIT_VERSION}\"."
    )
  endif()

  string(TIMESTAMP BUILD_STAMP "%Y-%m-%dT%H:%M:%SZ")

  # message("GIT_VERSION=\"${GIT_VERSION}\" BUILD_STAMP=\"${BUILD_STAMP}\"")

  target_compile_definitions(${TARGET} PRIVATE GIT_VERSION="${GIT_VERSION}"
                                               BUILD_STAMP="${BUILD_STAMP}")

endmacro()
