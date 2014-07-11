add_executable(test_omp main.cpp)
if(WIN32)
target_link_libraries(test_omp
lapack
)
else(WIN32)
target_link_libraries(test_omp
/usr/lib/liblapack.so.3gf
lapack
)
endif(WIN32)
