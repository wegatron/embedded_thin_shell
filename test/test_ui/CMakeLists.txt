SET(APPNAME test_ui)


include_directories(/usr/lib/openmpi/include)
link_directories(/usr/lib/openmpi/lib)


include_directories(

  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Common
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/ElasticModel
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/solid_elastic/Math
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/Animation
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/Manipulation
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/RecordAndPlay
  ${PROJECT_SOURCE_DIR}/src/volume_simulator/ui_lib/Render
  ${PROJECT_SOURCE_DIR}/src/interpolator
  ${PROJECT_SOURCE_DIR}/src/shell_deformer
)

SET(NONE_QT_CPP
  main.cpp
  PassiveObject.cpp
)

SET(UI_SOURCES
  DragNodeOp.cpp
  DataModel.cpp
  MainWindow.cpp  
)

SET(UI_HEADERS 
  DragNodeOp.h
  PerturbationOp.h
  SelectionOp.h
  MainWindow.h
  SelectionOp.h
  DataModelRender.h
  DataModel.h
)

SET(UI_FORMS 
  simulator.ui
)

file(GLOB interpolate_source ${PROJECT_SOURCE_DIR}/src/interpolator/*.cpp)
file(GLOB shell_deformer_source ${PROJECT_SOURCE_DIR}/src/shell_deformer/*.cpp
                                ${CMAKE_SOURCE_DIR}/src/parallize/linear_solver.cpp
                                ${CMAKE_SOURCE_DIR}/src/parallize/SQP.cpp)


QT4_WRAP_CPP(UI_HEADERS_MOC ${UI_HEADERS})
QT4_WRAP_UI(UI_FORMS_HEADERS ${UI_FORMS})


ADD_EXECUTABLE(${APPNAME}

  ${UI_HEADERS}
  ${NONE_QT_CPP}
  ${UI_SOURCES} 
  ${UI_HEADERS_MOC}
  ${UI_FORMS_HEADERS}
  ${interpolate_source}
  ${shell_deformer_source}
)
if(WIN32)
message(!!!!!!!!!!!!!Qt libraries is ${QT_LIBRARIES})
TARGET_LINK_LIBRARIES(${APPNAME}

  # ${CMAKE_SOURCE_DIR}/lib/release/libsolid_elastic.a
  ${LIBRARY_OUTPUT_PATH}/libsolid_elastic.a
  # ${CMAKE_SOURCE_DIR}/lib/release/libutility_ui.a
  ${LIBRARY_OUTPUT_PATH}/libutility_ui.a
  ${QGLViewer_LIBRARIES}
  ${QT_LIBRARIES}
  ${GLEW_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${Boost_LIBRARIES}

  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
  lapack
jtf-mesh
zjucad-tri_sub
hj_ANN_c
ann-parallel-static
#hj-util-static
jtf-optimizer
zjucad-ptree
shell
)
else(WIN32)
TARGET_LINK_LIBRARIES(${APPNAME}
mpi
petsc
petscmat
petscvec
petscksp
petscdm
  solid_elastic
  utility_ui

  ${QGLViewer_LIBRARIES}
  ${QT_LIBRARIES}
  ${GLEW_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${Boost_LIBRARIES}

  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
  lapack
jtf-mesh
zjucad-tri_sub
hj_ANN_c
#hj-util-static
jtf-optimizer
zjucad-optimizer
zjucad-ptree
shell
glut
)
endif(WIN32)
