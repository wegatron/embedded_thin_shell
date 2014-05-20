include_directories("../src")
include_directories("..")


INCLUDE_DIRECTORIES(

  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Common
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/ElasticModel
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Math
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/Animation
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/Manipulation
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/RecordAndPlay
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/Render
)
link_directories($ENV{HOME}/usr/lib)

# add_subdirectory(test_cluster)
# add_subdirectory(test_embed)
# add_subdirectory(test_shell)
# add_subdirectory(test_volume)
# add_subdirectory(test_interpolation)
# add_subdirectory(test_omp)
add_subdirectory(test_ui)
# add_subdirectory(test_mpi)
# add_subdirectory(test_ball_on_sofa)
# add_subdirectory(test_domain_decomposition)
# add_subdirectory(test_node_cpu)
# add_subdirectory(test_multi_threads)
# add_subdirectory(test_petsc)
# add_subdirectory(test_gen_data)
