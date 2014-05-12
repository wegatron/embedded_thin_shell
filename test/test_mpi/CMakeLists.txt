include_directories(/usr/lib/openmpi/include)
include_directories(/usr/lib/openmpi/include/openmpi)
link_directories(/usr/lib/openmpi/lib)

#set(CMAKE_CXX_FLAGS "-I/usr/lib/openmpi/include -I/usr/lib/openmpi/include/openmpi -pthread -L/usr/lib/openmpi/lib -lmpi_cxx -lmpi -lopen-rte -lopen-pal -ldl -Wl,--export-dynamic -lnsl -lutil -lm -ldl ${CMAKE_CXX_FLAGS}")

add_executable(test_mpi main.cpp)
target_link_libraries(test_mpi 
mpi_cxx
mpi
open-rte
open-pal
dl
nsl
)
