add_test(NAME cli-info COMMAND info --build)

add_test(NAME cli-plugins COMMAND plugins)

add_test(NAME cli-schema COMMAND schema sim)

add_test(NAME cli-echo
         COMMAND echo scenario ${CMAKE_CURRENT_SOURCE_DIR}/yaml/scenario/cross.yaml)

add_test(NAME cli-sample
         COMMAND sample ${CMAKE_CURRENT_SOURCE_DIR}/yaml/scenario/cross.yaml)

add_test(NAME cli-sample-uniform
         COMMAND sample "{sampler: uniform, from: 0, to: 10}" --type int --number 10)


add_test(NAME cli-run
         COMMAND run ${CMAKE_CURRENT_SOURCE_DIR}/yaml/experiment/cross.yaml)
