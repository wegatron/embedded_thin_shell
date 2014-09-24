SET(LIBRARY_NAME solid_elastic)

AUX_SOURCE_DIRECTORY(. simulator_src)
AUX_SOURCE_DIRECTORY(./Common solid_common_src)
AUX_SOURCE_DIRECTORY(./Math solid_math_src)
AUX_SOURCE_DIRECTORY(./ElasticModel solid_elastic_src)

INCLUDE_DIRECTORIES(./Common)
INCLUDE_DIRECTORIES(./ElasticModel)
INCLUDE_DIRECTORIES(./Math)

file(GLOB header *.h)
file(GLOB source *.cpp)

ADD_LIBRARY(${LIBRARY_NAME}
  ${header}
  ${simulator_src}
  ${solid_elastic_src}
  ${solid_math_src}
  ${solid_common_src}
  )

add_subdirectory(ElasticModel/collision)
