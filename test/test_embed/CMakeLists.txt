file(GLOB source "../../src/interpolator/*.cpp")


add_executable(test_embed main.cpp ${source})
target_link_libraries(test_embed
jtf-mesh
zjucad-tri_sub
blas
lapack
hj_ANN_c
${Boost_LIBRARIES}
)
