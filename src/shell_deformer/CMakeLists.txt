project(shell_deformer)
cmake_minimum_required(VERSION 2.8)

file(GLOB shell_deformer_header *.h)
file(GLOB shell_deformer_source *.cpp)

link_directories($ENV{HOME}/usr/lib)

add_library_from_dir(shell_deformer SHARED ./)
target_link_libraries(shell_deformer
jtf-optimizer
jtf-mesh
optimized
zjucad-ptree
#hj-util-static debug hj-util-staticd
hj-function
shell
gomp
)
