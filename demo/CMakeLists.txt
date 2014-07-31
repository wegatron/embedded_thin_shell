#include_directories(/usr/local/include/utility)

include_directories("../src")
include_directories(
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Common
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/ElasticModel
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Math
)

file(GLOB parallize_source ${CMAKE_SOURCE_DIR}/src/parallize/*.cpp)
file(GLOB demo_source ./*.cpp)
add_executable(collision_demo  ${parallize_source} ${demo_source})

target_link_libraries(collision_demo
  interpolator
  solid_elastic
jtf-mesh
zjucad-tri_sub
hj_ANN_c
hj-util-static
jtf-optimizer
zjucad-optimizer
shell_deformer

shell
  ${Boost_LIBRARIES}
  ${ARPACK_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${LAPACK_LIBRARIES}
  mpi
petsc
petscmat
petscvec
petscksp
petscdm
metis
#zjucad-ptree
)
