



file(GLOB interpolator_src ${CMAKE_SOURCE_DIR}/src/interpolator/*.cpp)



if(WIN32)
add_executable(test_interpolation main.cpp ${interpolator_src})
target_link_libraries(test_interpolation
jtf-mesh
zjucad-tri_sub
blas
lapack
hj_ANN_c
ann-parallel-static
#hj-util-static
)
else(WIN32)
target_link_libraries(test_interpolation
jtf-mesh
zjucad-tri_sub
blas
#lapack
lapack
hj_ANN_c
#hj-util-static
)
endif(WIN32)
