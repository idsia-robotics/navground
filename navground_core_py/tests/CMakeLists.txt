add_test(NAME cli-info COMMAND ${Python3_EXECUTABLE} -m navground.core info
                               --build)

add_test(NAME cli-plugins COMMAND ${Python3_EXECUTABLE} -m navground.core
                                  plugins)

add_test(NAME cli-schema COMMAND ${Python3_EXECUTABLE} -m navground.core schema
                                 core)

add_test(NAME cli-echo
         COMMAND ${Python3_EXECUTABLE} -m navground.core echo behavior
                 ${CMAKE_CURRENT_SOURCE_DIR}/yaml/behavior/dummy.yaml)

add_test(NAME properties COMMAND ${PYTHON_EXECUTABLE} -m pytest
                                 ${CMAKE_CURRENT_SOURCE_DIR}/test_properties.py)
