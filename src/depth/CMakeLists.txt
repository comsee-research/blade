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

#--------------------
# DEPENDENCY: OpenMP (optional)
#--------------------
set(USE_OPEN_MP FALSE CACHE BOOL "Set to TRUE to use OpenMP")
if (USE_OPEN_MP)
	find_package(OpenMP)
	if (OPENMP_FOUND)
		add_definitions(-fopenmp -DHAVE_OPENMP)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		if (CMAKE_COMPILER_IS_GNUCC)
			set(EXTERNAL_LIBS ${EXTERNAL_LIBS} gomp)
    	message("-- OpenMP found, parallel computer enabled")
		endif(CMAKE_COMPILER_IS_GNUCC)
	endif(OPENMP_FOUND)
endif (USE_OPEN_MP)

##INCLUDE DIRECTORIES
set(MULTIFOCUS_INCDIRS "src" "geometry" "graphic" "io" "optimization" "processing")

##SOURCES
set(MULTIFOCUS_SRCS 
	##---------------
	src/optimization/depth.cpp
	src/optimization/functions.cpp
	src/optimization/errors/disparity.cpp
	src/optimization/errors/scaling.cpp
	src/optimization/errors/lidarcamera.cpp
	##---------------
	src/processing/depth/depth.cpp
	src/processing/depth/filter.cpp
	src/processing/depth/neighbors.cpp
	src/processing/depth/pairing.cpp
	src/processing/depth/search.cpp
	src/processing/depth/initialization.cpp
	src/processing/depth/export.cpp
	src/processing/depth/compute.cpp
	src/processing/depth/strategy.cpp
	##---------------
	src/processing/calibration/scaling.cpp
	src/processing/calibration/lidarcamera.cpp
	##---------------
)

message(${LIBPLENO_LIBRARIES})
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${LIBPLENO_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

##################################################
##################################################
set(APP_DEPTH_SRCS src/apps/depth/depth.cpp src/apps/depth/utils.cpp)
add_executable(depth ${APP_DEPTH_SRCS} ${MULTIFOCUS_SRCS})
target_include_directories(${ProjectId} PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(${ProjectId} ${MULTIFOCUS_LIBS} pthread)


##################################################
##################################################
set(APP_DEPTH_FROM_OBS_SRCS src/apps/depth/depth_from_observations.cpp src/apps/depth/utils.cpp)
add_executable(depth_from_obs ${APP_DEPTH_FROM_OBS_SRCS} ${MULTIFOCUS_SRCS})
target_include_directories(depth_from_obs PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(depth_from_obs ${MULTIFOCUS_LIBS} pthread)


##################################################
##################################################
set(APP_EXPORT_SRCS src/apps/export/depth_export.cpp src/apps/export/utils.cpp)
add_executable(depth_export ${APP_EXPORT_SRCS} ${MULTIFOCUS_SRCS})
target_include_directories(depth_export PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(depth_export ${MULTIFOCUS_LIBS} pthread)


##################################################
##################################################
set(APP_CALIBRATE_SRCS src/apps/calibrate/calibrate.cpp src/apps/calibrate/utils.cpp src/apps/calibrate/eval.cpp)
add_executable(depth_calibrate ${APP_CALIBRATE_SRCS} ${MULTIFOCUS_SRCS})
target_include_directories(depth_calibrate PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(depth_calibrate ${MULTIFOCUS_LIBS} pthread)


##################################################
##################################################
set(APP_LIDARCAMERA_SRCS src/apps/lidarcamera/lidarcamera.cpp src/apps/lidarcamera/utils.cpp)
add_executable(depth_lidarcamera ${APP_LIDARCAMERA_SRCS} ${MULTIFOCUS_SRCS})
target_include_directories(depth_lidarcamera PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(depth_lidarcamera ${MULTIFOCUS_LIBS} pthread)

##################################################
##################################################
set(APP_DISTANCES_SRCS src/apps/distances/distances.cpp src/apps/distances/utils.cpp)
add_executable(depth_distances ${APP_DISTANCES_SRCS} ${MULTIFOCUS_SRCS})
target_include_directories(depth_distances PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(depth_distances ${MULTIFOCUS_LIBS} pthread)

##################################################
##################################################
##SOURCES
set(APP_EVALUATE_SRCS 
	##---------------
	src/apps/evaluate/depth_evaluate.cpp
	src/apps/evaluate/utils.cpp
	##---------------
	src/apps/evaluate/reduce.cpp
	src/apps/evaluate/compute.cpp
	src/apps/evaluate/export.cpp
	##---------------
)

add_executable(depth_evaluate ${APP_EVALUATE_SRCS} ${MULTIFOCUS_SRCS})
target_include_directories(depth_evaluate PRIVATE ${MULTIFOCUS_INCDIRS})
target_link_libraries(depth_evaluate ${MULTIFOCUS_LIBS} pthread)
