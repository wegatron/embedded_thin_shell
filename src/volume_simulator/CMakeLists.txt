project(SolidShellSimulator C CXX Fortran)
cmake_minimum_required(VERSION 2.8)

add_definitions(-DPROJ_PATH="${CMAKE_SOURCE_DIR}/")
add_definitions(-DTEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test_case/TestData/")

# debug and release
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -DHAVE_NAMESPACES  -ldl -pg")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -DHAVE_NAMESPACES -ldl")

# print log informations
add_definitions(-DLOG_INFO)
add_definitions(-DLOG_ERROR)
add_definitions(-DLOG_WARN)
add_definitions(-DLOG_TRACE)
add_definitions(-DLOG_DEBUG)

# check release or debug version
if(NOT CMAKE_BUILD_TYPE)
  message("generate release version")
  set(CMAKE_BUILD_TYPE "release")
  set(CMAKE_BUILD_TYPE_DIR "release")

elseif(CMAKE_BUILD_TYPE STREQUAL "RELEASE")
  message("generate release version")
  set(CMAKE_BUILD_TYPE "release")
  set(CMAKE_BUILD_TYPE_DIR "release")

elseif(${CMAKE_BUILD_TYPE} STREQUAL "DEBUG")
  message("generate debug version")
  set(CMAKE_BUILD_TYPE_DIR "debug")
  set(CMAKE_BUILD_TYPE "debug")
  
endif(NOT CMAKE_BUILD_TYPE)

message("CMAKE_CXX_FLAGS_DEBUG: " ${CMAKE_CXX_FLAGS_DEBUG})
message("CMAKE_CXX_FLAGS_RELEASE: " ${CMAKE_CXX_FLAGS_RELEASE})

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE_DIR})

#message(${CMAKE_BINARY_DIR})
#message(${PROJECT_BINARY_DIR})

include_directories(

  ${PROJECT_SOURCE_DIR}/solid_elastic
  ${PROJECT_SOURCE_DIR}/solid_elastic/Common
  ${PROJECT_SOURCE_DIR}/solid_elastic/ElasticModel
  ${PROJECT_SOURCE_DIR}/solid_elastic/Math
  ${PROJECT_SOURCE_DIR}/ui_lib
  ${PROJECT_SOURCE_DIR}/ui_lib/Animation
  ${PROJECT_SOURCE_DIR}/ui_lib/Manipulation
  ${PROJECT_SOURCE_DIR}/ui_lib/RecordAndPlay
  ${PROJECT_SOURCE_DIR}/ui_lib/Render

)


# PROJECTs
add_subdirectory(solid_elastic)
add_subdirectory(solid_tools)
add_subdirectory(ui_lib)
