cmake_minimum_required(VERSION 2.8)

get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${ProjectId})

message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
message("BLADE: Blur Aware Depth Estimation with a Multi-focus Plenoptic camera")
message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

add_subdirectory(src/depth)
add_subdirectory(src/distances)
add_subdirectory(src/evaluate)
add_subdirectory(src/export)
add_subdirectory(src/lidarcamera)
add_subdirectory(src/scaling)


