add_test(NAME cli-info COMMAND info --build)

add_test(NAME cli-plugins COMMAND plugins)

add_test(NAME cli-schema COMMAND schema core)

add_test(NAME cli-echo
         COMMAND echo behavior ${CMAKE_CURRENT_SOURCE_DIR}/yaml/behavior/dummy.yaml)
