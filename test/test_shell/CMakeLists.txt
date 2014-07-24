

file(GLOB source "../../src/shell_deformer/*.cpp" "../../src/parallize/*.cpp")

link_directories($ENV{HOME}/usr/Linux/64/lib)
link_directories($ENV{HOME}/usr/Linux/64/gcc4/lib)
link_directories($ENV{HOME}/usr/lib)
include_directories(/usr/lib/openmpi/include)
link_directories(/usr/lib/openmpi/lib)

add_executable(test_shell main.cpp ${source})
target_link_libraries(test_shell
mpi
petsc
petscmat
petscvec
petscksp
petscdm
jtf-mesh
#hj-util-static
jtf-optimizer
zjucad-optimizer
zjucad-ptree
  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
blas
shell
lapack
metis
)
