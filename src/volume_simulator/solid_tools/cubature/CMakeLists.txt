SET(APP_NAME cubature)

AUX_SOURCE_DIRECTORY(. cubature_src)
AUX_SOURCE_DIRECTORY(./cubacode cubacode_src)

ADD_EXECUTABLE(${APP_NAME} 
  ${cubature_src}
  ${cubacode_src}
  )

TARGET_LINK_LIBRARIES(${APP_NAME}

  solid_elastic
  ${LAPACK_LIBRARIES}
  ${Boost_LIBRARIES}
  )
