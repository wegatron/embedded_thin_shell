project(embedded_thin_shell)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_VERBOSE_MAKEFILE on)

if(WIN32)
	include($ENV{HOME}/usr/share/cmake/Modules/win32_default.cmake)
endif(WIN32)

include($ENV{HOME}/usr/share/cmake/Modules/geo_sim_sdk.cmake)

include_geo_sim_sdk()
link_geo_sim_sdk()

link_directories($ENV{HOME}/usr/lib)

#############################################################################
########################find packages########################################
set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/;${CMAKE_MODULE_PATH}" )
# if(WIN32)
	# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DWIN32 ")
	# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -DWIN32 ")
	# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DWIN32 ")
	# set(CMAKE_C_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -DWIN32 ")
# endif(WIN32)
find_package(OpenGL REQUIRED)

find_package(GLEW REQUIRED)

# set(Boost_DEBUG ON)

set(Boost_USE_STATIC_LIBS        ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package( Boost COMPONENTS filesystem system REQUIRED)

if(Boost_FOUND)
include_directories(${Boost_INCLUDE_DIRS})
else(Boost_FOUND)
    message("CAN NOT FIND BOOST")
endif(Boost_FOUND)

if(WIN32)
# for lapack and blas
  link_directories($ENV{HOME}/usr/lib64/lapack_blas_windows)
# in windows we use superlu 4.3
  set(SUPERLU2.0_LIBRARIES $ENV{HOME}/usr/lib/libsuperlu_4.3.a)
# for arpack and arpack++
  set(ARPACK_LIBRARIES $ENV{HOME}/usr/lib/libarpack_Win64.a)
  include_directories($ENV{HOME}/usr//include/arpack++)
# for eigen
  include_directories($ENV{HOME}/usr/include)
  include_directories($ENV{HOME}/usr/include/eigen3)
  
  set(SUITESPARSE_INCLUDE_DIRS $ENV{HOME}/usr/include/suitesparse)
  include_directories(${SUITESPARSE_INCLUDE_DIRS})
  link_directories($ENV{HOME}/usr/lib64)
  set(SUITESPARSE_LIBRARIES  umfpack amd SuiteSparse_config cholmod btf ccolamd  colamd ldl spqr camd klu metis blas lapack)
else(WIN32)
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
  
  find_package(Eigen3 REQUIRED)
  if(EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
  endif(EIGEN3_FOUND)
  
  find_package(SuiteSparse REQUIRED)
  if(SUITESPARSE_FOUND)
     include_directories(${SUITESPARSE_INCLUDE_DIRS})
  else(SUITESPARSE_FOUND)
     message("!!!! ERROR SuiteSparse not found!")
  endif(SUITESPARSE_FOUND)
endif(WIN32)

set(Qt4_DEBUG ON)
#set(QT_USE_IMPORTED_TARGETS ON)
if(WIN32)
	include_directories("C:/libQGLViewer")
	include_directories("C:/libQGLViewer/QGLViewer")
	set(QGLViewer_LIBRARIES "C:/libQGLViewer/lib/libQGLViewer2.a")
else(WIN32)
	find_package(QGLViewer)
        #INCLUDE PETSC
        include_directories(/usr/include/petsc)
endif(WIN32)

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

