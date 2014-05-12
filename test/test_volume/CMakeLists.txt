SET(APP_NAME test_wrikle_offline)

AUX_SOURCE_DIRECTORY(. hello_dino_src)

file(GLOB interpolator_src ${CMAKE_SOURCE_DIR}/src/interpolator/*.cpp)
file(GLOB deformer_src ${CMAKE_SOURCE_DIR}/src/shell_deformer/*.cpp ${CMAKE_SOURCE_DIR}/src/parallize/*.cpp)

include_directories(/usr/lib/openmpi/include)
link_directories(/usr/lib/openmpi/lib)

ADD_EXECUTABLE(${APP_NAME}
  ${hello_dino_src}
  ${interpolator_src}
  ${deformer_src}
  )

TARGET_LINK_LIBRARIES(${APP_NAME}
mpi
petsc
petscmat
petscvec
petscksp
petscdm
  solid_elastic
  ${Boost_LIBRARIES}
  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
  ${Lapack_LIBRARIES}
jtf-mesh
zjucad-tri_sub
hj_ANN_c
hj-util-static
jtf-optimizer
zjucad-optimizer
zjucad-ptree
shell
/usr/lib/liblapack.so.3gf
metis
)
