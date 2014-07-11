project(interpolator)
cmake_minimum_required(VERSION 2.8)

file(GLOB header *.h)
file(GLOB source *.cpp)

add_library(interpolator SHARED ${header} ${source})

if(WIN32)
target_link_libraries(interpolator
jtf-mesh
hj_ANN_c
ann-parallel-static
zjucad-tri_sub
lapack
)
else(WIN32)
target_link_libraries(interpolator
jtf-mesh
hj_ANN_c
zjucad-tri_sub
lapack
)
endif(WIN32)