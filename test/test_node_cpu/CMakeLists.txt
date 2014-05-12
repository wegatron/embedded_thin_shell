file(GLOB cluster_source ${CMAKE_SOURCE_DIR}/src/shell_deformer/*.cpp)
file(GLOB parallize_source ${CMAKE_SOURCE_DIR}/src/parallize/*.cpp)

link_directories(/usr/lib)

add_executable(test_node_cpu main.cpp ${cluster_source} ${parallize_source})
target_link_libraries(test_node_cpu
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
zjucad-optimizer
shell
metis
  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
/usr/lib/libmpi.so.0
)

