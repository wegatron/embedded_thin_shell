project(interpolator)
cmake_minimum_required(VERSION 2.8)

file(GLOB header *.h)
file(GLOB source *.cpp)

add_library(interpolator SHARED ${header} ${source})
