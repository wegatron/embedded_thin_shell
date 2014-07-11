file(GLOB source "../../src/interpolator/*.cpp")


if(WIN32)
target_link_libraries(test_embed
jtf-mesh
zjucad-tri_sub
blas
lapack
hj_ANN_c
ann-parallel-static
${Boost_LIBRARIES}
)
else(WIN32)
target_link_libraries(test_embed
jtf-mesh
zjucad-tri_sub
blas
#lapack
lapack
hj_ANN_c
${Boost_LIBRARIES}
)
endif(WIN32)
