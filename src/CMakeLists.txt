
file(GLOB conf_header *.h)
add_custom_target(conf SOURCES ${conf_header})

add_subdirectory(volume_simulator)
add_subdirectory(shell_deformer)
add_subdirectory(interpolator)
add_subdirectory(parallize)

