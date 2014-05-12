SET(APPNAME CommonTest)

AUX_SOURCE_DIRECTORY(. CommonTest_src)

ADD_EXECUTABLE(${APPNAME} ${CommonTest_src})

TARGET_LINK_LIBRARIES( ${APPNAME} 
  solid_elastic
  ${Boost_LIBRARIES}
  )
