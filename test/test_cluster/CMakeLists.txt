file(GLOB deform_cpp "../../src/shell_deformer/cluster.cpp" "../../src/interpolator/interpolator.cpp")

add_executable(test_cluster main ${deform_cpp})
target_link_libraries(test_cluster
jtf-mesh
zjucad-tri_sub
hj_ANN_c
    ${LAPACK_LIBRARIES}
${Boost_libraries}
)

#add_subdirectory()
