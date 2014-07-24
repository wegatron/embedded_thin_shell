file(GLOB cluster_source ${CMAKE_SOURCE_DIR}/src/shell_deformer/*.cpp)
file(GLOB parallize_source ${CMAKE_SOURCE_DIR}/src/parallize/*.cpp)

include_directories(/usr/lib/openmpi/include)
link_directories(/usr/lib/openmpi/lib)
#link_directories(/usr/local/lib)


add_executable(test_multi_threads main.cpp ${cluster_source} ${parallize_source})
target_link_libraries(test_multi_threads
mpi
petsc
petscmat
petscvec
petscksp
petscdm
jtf-mesh
jtf-mesh
zjucad-tri_sub
hj_ANN_c
hj-util-static
jtf-optimizer
zjucad-ptree
#zjucad-optimizer
shell
metis
${ARPACK_LIBRARIES}
${SUITESPARSE_LIBRARIES}
${SUPERLU2.0_LIBRARIES}
blas
lapack
)
