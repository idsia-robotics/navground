add_test(NAME collision COMMAND collision)
add_test(NAME collision_cached COMMAND collision_cached)

add_test(NAME behavior COMMAND behavior)
add_test(NAME behavior_yaml COMMAND behavior_yaml "{type: HL}")

add_test(NAME controller COMMAND controller)
add_test(NAME controller_3d COMMAND controller_3d)
add_test(NAME controller_async COMMAND controller_async)

add_test(NAME world COMMAND world)
add_test(NAME world_group COMMAND world_group)
add_test(NAME world_yaml COMMAND world_yaml)

add_test(NAME scenario_group COMMAND scenario_group)
add_test(NAME scenario_registered COMMAND scenario_registered)
add_test(NAME scenario_yaml COMMAND scenario_yaml)

add_test(NAME experiment COMMAND experiment)
add_test(NAME run COMMAND run)





