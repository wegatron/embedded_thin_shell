SET(LIBRARY_NAME utility_ui)

SET(NONE_QT_CPP

  RecordAndPlay/EventSerialization.cpp
  Render/SelfRenderRect.cpp
  Render/FileDialog.cpp
  Render/MeshRender.cpp
  Render/VolNodeGroupRender.cpp
  Render/DragGroupSelDraw.cpp
  Render/AxisTorus.cpp
)

SET(UI_SOURCES

  RecordAndPlay/QInputEventRecorder.cpp
  RecordAndPlay/RecordAndReplayWrapper.cpp
  Manipulation/SelectCtrl.cpp
  Manipulation/DragCtrl.cpp
  Animation/AniCtrl.cpp
  Manipulation/ConTrackBall.cpp
  Manipulation/Manipulatoion.cpp
  Render/QGLViewerExt.cpp
)

SET(UI_HEADERS 

  RecordAndPlay/QInputEventRecorder.h
  RecordAndPlay/RecordAndReplayWrapper.h
  Manipulation/SelectCtrl.h
  Manipulation/DragCtrl.h
  Manipulation/VolObjCtrl.h
  Animation/AniCtrl.h
  Animation/AniSliderBar.h
  Render/QGLViewerExt.h
  Manipulation/ConTrackBall.h
  Manipulation/Manipulatoion.h
)

SET(UI_FORMS 

)

QT4_WRAP_CPP(UI_HEADERS_MOC ${UI_HEADERS})
QT4_WRAP_UI(UI_FORMS_HEADERS ${UI_FORMS})

ADD_LIBRARY(${LIBRARY_NAME}

  ${NONE_QT_CPP}
  ${UI_SOURCES}
  ${UI_HEADERS_MOC}
  ${UI_FORMS_HEADERS}
)