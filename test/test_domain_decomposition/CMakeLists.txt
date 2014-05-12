

file(GLOB cluster_source ${CMAKE_SOURCE_DIR}/src/shell_deformer/cluster.cpp)
file(GLOB parallize_source ${CMAKE_SOURCE_DIR}/src/parallize/*.cpp)

add_executable(test_domain_decomposition main.cpp ${cluster_source} ${parallize_source})
target_link_libraries(test_domain_decomposition
petsc
petscmat
petscvec
petscksp
petscdm
jtf-mesh
hj-util
zjucad-optimizer
zjucad-ptree
metis
  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
/usr/lib/libmpi.so.0
)
