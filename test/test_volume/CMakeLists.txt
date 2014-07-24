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

if(WIN32)
TARGET_LINK_LIBRARIES(${APP_NAME}
  # ${CMAKE_SOURCE_DIR}/lib/release/libsolid_elastic.a
  ${LIBRARY_OUTPUT_PATH}/libsolid_elastic.a
  ${Boost_LIBRARIES}
  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
  lapack
jtf-mesh
zjucad-tri_sub
hj_ANN_c
ann-parallel-static
# hj-util-static
jtf-optimizer
zjucad-ptree
shell
)
else(WIN32)
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
  lapack
jtf-mesh
zjucad-tri_sub
hj_ANN_c
# hj-util-static
jtf-optimizer
zjucad-optimizer
zjucad-ptree
shell
blas
/usr/lib/liblapack.so.3gf
metis
)
endif(WIN32)
