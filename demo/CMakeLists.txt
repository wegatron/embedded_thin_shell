include_directories(/usr/local/include/utility)
include_directories("../src")
include_directories(
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Common
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/ElasticModel
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Math
)

add_executable_from_dir(collision_demo .)

target_link_libraries(collision_demo
  interpolator
  solid_elastic
  ${Boost_LIBRARIES}

  ${LAPACK_LIBRARIES}
  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}

jtf-mesh
zjucad-tri_sub
hj_ANN_c
hj-util-static
jtf-optimizer
zjucad-optimizer
shell_deformer
shell
#zjucad-ptree
)
