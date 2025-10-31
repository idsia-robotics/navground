set(prefix ${Python_EXECUTABLE} -m navground.core)

add_test(NAME cli-info COMMAND ${prefix} info --build)

add_test(NAME cli-plugins COMMAND ${prefix} plugins)

add_test(NAME cli-schema COMMAND ${prefix} schema core)

add_test(NAME cli-echo
         COMMAND ${prefix} echo behavior
                 ${CMAKE_CURRENT_SOURCE_DIR}/yaml/behavior/dummy.yaml)

add_test(NAME properties COMMAND ${Python_EXECUTABLE} -m pytest
                                 ${CMAKE_CURRENT_SOURCE_DIR}/test_properties.py)
