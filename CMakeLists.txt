project(embedded_thin_shell)
cmake_minimum_required(VERSION 2.8)

include($ENV{HOME}/usr/share/cmake/Modules/geo_sim_sdk.cmake)

include_geo_sim_sdk()
link_geo_sim_sdk()
link_directories($ENV{HOME}/usr/lib)

#############################################################################
########################find packages########################################
set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/;${CMAKE_MODULE_PATH}" )

find_package(OpenGL REQUIRED)
find_package(GLEW)

set(Boost_USE_STATIC_LIBS        ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package( Boost COMPONENTS filesystem system REQUIRED)

set(BLA_VENDOR "Goto")
find_package(BlasLapack)
if(NOT LAPACK_FOUND)
  set(BLA_VENDOR "OpenBlas")
  find_package(BlasLapack)
endif(NOT LAPACK_FOUND)
if(NOT LAPACK_FOUND)
  set(BLA_VENDOR "Generic")
  find_package(BlasLapack)
endif(NOT LAPACK_FOUND)

find_package(SuperLU2.0 REQUIRED)
find_package(Arpack REQUIRED)
find_package(Arpack++ REQUIRED)
if(Arpack++_FOUND)
  include_directories(${ARPACK++_INCLUDE_DIR})
endif(Arpack++_FOUND)

find_package(SuiteSparse REQUIRED)
if(SUITESPARSE_FOUND)
  include_directories(${SUITESPARSE_INCLUDE_DIRS})
endif(SUITESPARSE_FOUND)

find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(EIGEN3_FOUND)

#INCLUDE PETSC
include_directories(/usr/include/petsc)

#####################################QT#####################################
find_package(Qt4 COMPONENTS QtCore QtGui QtXml QtOpenGL REQUIRED )
find_package(QGLViewer)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
include_directories(${CMAKE_BINARY_DIR})
#####################################QT#####################################
###########################################################################


set(CMAKE_CXX_FLAGS "-fopenmp -fpermissive -std=c++0x ${CMAKE_CXX_FLAGS}")

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(common)

