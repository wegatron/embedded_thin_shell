add_executable(test_omp main.cpp)
target_link_libraries(test_omp
#${Lapack_LIBRARIES}
lapack
)
