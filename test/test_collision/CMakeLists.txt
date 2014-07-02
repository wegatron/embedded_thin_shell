ADD_EXECUTABLE(test_drop
        drop_case.cpp
        )
ADD_EXECUTABLE(test_hit_ball
        hit_ball.cpp
        )
ADD_EXECUTABLE(test_move_ball
        ball_move.cpp
)

ADD_EXECUTABLE(gen_obj_normal
genobj_normal.cpp
)

TARGET_LINK_LIBRARIES(gen_obj_normal
        solid_elastic
        ${Boost_LIBRARIES}
        jtf-mesh)


TARGET_LINK_LIBRARIES(test_move_ball
        solid_elastic
            ${Boost_LIBRARIES}

        )

TARGET_LINK_LIBRARIES(test_drop
  interpolator
    solid_elastic
    ${Boost_LIBRARIES}

  ${LAPACK_LIBRARIES}
  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}

jtf-mesh
zjucad-tri_sub
hj_ANN_c
hj-util-static
jtf-optimizer
zjucad-optimizer
shell_deformer
shell
#zjucad-ptree
)



TARGET_LINK_LIBRARIES(test_hit_ball
  interpolator
    solid_elastic
    ${Boost_LIBRARIES}

  ${LAPACK_LIBRARIES}
  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}

jtf-mesh
zjucad-tri_sub
hj_ANN_c
hj-util-static
jtf-optimizer
zjucad-optimizer
shell_deformer
shell
#zjucad-ptree
)
