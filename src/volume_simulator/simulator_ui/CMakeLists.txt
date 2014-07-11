SET(APPNAME simulator_ui)

SET(NONE_QT_CPP
  main.cpp
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

QT4_WRAP_CPP(UI_HEADERS_MOC ${UI_HEADERS})
QT4_WRAP_UI(UI_FORMS_HEADERS ${UI_FORMS})

ADD_EXECUTABLE(${APPNAME}

  ${NONE_QT_CPP}
  ${UI_SOURCES} 
  ${UI_HEADERS_MOC}
  ${UI_FORMS_HEADERS}
)

if(WIN32)

TARGET_LINK_LIBRARIES(${APPNAME}
  
  # ${LIBRARY_OUTPUT_PATH}/release/libsolid_elastic.a
  # ${LIBRARY_OUTPUT_PATH}/release/libutility_ui.a
  ${LIBRARY_OUTPUT_PATH}/libsolid_elastic.a
  ${LIBRARY_OUTPUT_PATH}/libutility_ui.a
  ${QGLViewer_LIBRARIES}
  #${QT_DLL_LIBRARIES}
  # C:/Qt4.8.5/bin/QtOpenGL4.dll
  # C:/Qt4.8.5/bin/QtGui4.dll
  # C:/Qt4.8.5/bin/QtXml4.dll
  # C:/Qt4.8.5/bin/QtCore4.dll
  ${QT_LIBRARIES}
  #C:/MinGW/MinGW32/msys/1.0/home/wegatron/3rdlibs/qt-everywhere-opensource-src-4.8.5/lib/libQtCore4.a
  
  ${GLEW_LIBRARY}
  ${OPENGL_LIBRARIES}
  ${Boost_LIBRARIES}

  ${ARPACK_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${SUPERLU2.0_LIBRARIES}
  lapack
)
else(WIN32)
TARGET_LINK_LIBRARIES(${APPNAME}
  
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
)
endif(WIN32)