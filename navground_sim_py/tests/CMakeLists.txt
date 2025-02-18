set(prefix ${Python3_EXECUTABLE} -m navground.sim)
set(root ${CMAKE_CURRENT_SOURCE_DIR}/yaml)

add_test(NAME cli-info COMMAND ${prefix} info --build)

add_test(NAME cli-plugins COMMAND ${prefix} plugins)

add_test(NAME cli-schema COMMAND ${prefix} schema sim)

add_test(NAME cli-echo COMMAND ${prefix} echo scenario
                               ${root}/scenario/cross.yaml)

add_test(NAME cli-sample COMMAND ${prefix} sample ${root}/scenario/cross.yaml)

add_test(NAME cli-run COMMAND ${prefix} run ${root}/experiment/cross.yaml)

add_test(NAME cli-video
         COMMAND ${prefix} record_video ${root}/experiment/cross.yaml
                 /tmp/video.mp4 --factor 100)
