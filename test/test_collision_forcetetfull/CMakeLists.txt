add_executable(test_collisionBallForceTetFull
        main.cpp)

target_link_libraries(test_collisionBallForceTetFull
    solid_elastic
    collision_energy
    ${Boost_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${ARPACK_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    ${SUPERLU2.0_LIBRARIES}
    )
