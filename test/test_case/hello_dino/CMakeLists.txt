SET(APP_NAME hello_dino)

AUX_SOURCE_DIRECTORY(. hello_dino_src)

ADD_EXECUTABLE(${APP_NAME}
  ${hello_dino_src}
  )

message(${Boost_LIBRARIES})

TARGET_LINK_LIBRARIES(${APP_NAME}

  solid_elastic

  ${Boost_LIBRARIES}
  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
  ${LAPACK_LIBRARIES}
)
