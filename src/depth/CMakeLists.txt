cmake_minimum_required(VERSION 2.8)

get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${ProjectId})

message("-----------------------------------------------------------------------------------------")
message("${PROJECT_NAME}")
message("-----------------------------------------------------------------------------------------")

set(CMAKE_CXX_STANDARD 17)

find_package(libpleno REQUIRED)
find_package(Boost COMPONENTS program_options filesystem REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
add_definitions(-O3)

##LINK LIBRARIES
set(MULTIFOCUS_LIBS
	${LIBPLENO_LIBRARIES}
	${Boost_LIBRARIES}
)

##INCLUDE DIRECTORIES
set(MULTIFOCUS_INCDIRS "src")

##SOURCES
set(MULTIFOCUS_SRCS 
	src/utils.cpp
	##---------------
	src/geometry/depth/RawCoarseDepthMap.cpp
	##---------------
	src/optimization/depth.cpp
	src/optimization/errors/blurawaredisp.cpp
	src/optimization/errors/blurequalizationdisp.cpp
	src/optimization/errors/disparity.cpp
	##---------------
	src/processing/depth/depth.cpp
	src/processing/depth/depth_from_observations.cpp
	src/processing/depth/filter.cpp
	src/processing/depth/neighbors.cpp
	src/processing/depth/initialization.cpp
	src/processing/depth/export_cost_function.cpp
)

message(${LIBPLENO_LIBRARIES})
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${LIBPLENO_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

##################################################
##################################################
add_executable(${ProjectId} src/depth.cpp ${MULTIFOCUS_SRCS})
target_include_directories(${ProjectId} PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(${ProjectId} ${MULTIFOCUS_LIBS} pthread)


##################################################
##################################################
add_executable(depth_from_obs src/depth_from_observations.cpp ${MULTIFOCUS_SRCS})
target_include_directories(depth_from_obs PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(depth_from_obs ${MULTIFOCUS_LIBS} pthread)


##################################################
##################################################
add_executable(depth_export src/depth_export.cpp ${MULTIFOCUS_SRCS})
target_include_directories(depth_export PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(depth_export ${MULTIFOCUS_LIBS} pthread)
