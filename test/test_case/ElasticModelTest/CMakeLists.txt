SET(APPNAME ElasticModelTest)

AUX_SOURCE_DIRECTORY(. ElasticModel_src)

ADD_EXECUTABLE(${APPNAME} ${ElasticModel_src})

TARGET_LINK_LIBRARIES( ${APPNAME} 
  solid_elastic
  ${Boost_LIBRARIES}
  )
