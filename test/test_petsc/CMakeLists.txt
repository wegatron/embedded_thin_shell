

#include_directories($ENV{HOME}/usr/petsc-3.4.4/include)

#set(CMAKE_CXX_FLAGS "-I/usr/lib/openmpi/include -I/usr/lib/openmpi/include/openmpi -pthread -L/usr/lib/openmpi/lib -lmpi_cxx -lmpi -lopen-rte -lopen-pal -ldl -Wl,--export-dynamic -lnsl -lutil -lm -ldl ${CMAKE_CXX_FLAGS}")

include_directories(/usr/lib/openmpi/include)

add_executable(test_petsc main.cpp)
target_link_libraries(test_petsc
#$ENV{HOME}/usr/petsc-3.4.4/arch-linux2-c-debug/lib/libmpich.so
#$ENV{HOME}/usr/petsc-3.4.4/arch-linux2-c-debug/lib/libmpichcxx.so
#$ENV{HOME}/usr/petsc-3.4.4/arch-linux2-c-debug/lib/libfblas.a
#$ENV{HOME}/usr/petsc-3.4.4/arch-linux2-c-debug/lib/libflapack.a
#/usr/lib/libpetsc.so
mpi
petsc
petscmat
petscvec
petscksp
petscdm
)
