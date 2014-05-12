INCLUDE_DIRECTORIES(

  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Common
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/ElasticModel
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Math
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/Animation
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/Manipulation
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/RecordAndPlay
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/Render
)

ADD_SUBDIRECTORY(hello_dino)
#ADD_SUBDIRECTORY(CommonTest)
#ADD_SUBDIRECTORY(ElasticModelTest)
#ADD_SUBDIRECTORY(MathTest)
