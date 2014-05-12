SET(APP_NAME extend_basis)

AUX_SOURCE_DIRECTORY(. extend_basis_src)

ADD_EXECUTABLE(${APP_NAME} 
  ${extend_basis_src}
  )

TARGET_LINK_LIBRARIES(${APP_NAME}
  solid_elastic
  ${ARPACK_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${LAPACK_LIBRARIES}
  ${Boost_LIBRARIES}
  lapack
 )
