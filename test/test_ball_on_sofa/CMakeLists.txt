
# my bullet sdk is intalled under one dir which could be integrated in other project
set(BULLET_INCLUDE_DIR $ENV{HOME}/usr/libcode/Bullet/src)
set(BULLET_LINK_DIR $ENV{HOME}/usr/libcode/Bullet/lib)
include_directories(${BULLET_INCLUDE_DIR})
link_directories(${BULLET_LINK_DIR})
# end use bullet

aux_source_directory(. source)
add_executable(test_ball_on_sofa ${source})
target_link_libraries(test_ball_on_sofa
libBulletDynamics_gmake_x64_release.a
libBulletCollision_gmake_x64_release.a
libLinearMath_gmake_x64_release.a
${Boost_LIBRARIES}
)

