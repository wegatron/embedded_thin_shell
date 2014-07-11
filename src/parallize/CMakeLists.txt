file(GLOB header *.h)
file(GLOB source *.cpp)

include_directories(/usr/include/petsc)
add_custom_target(domain_decomposition SOURCES ${header} ${source})
