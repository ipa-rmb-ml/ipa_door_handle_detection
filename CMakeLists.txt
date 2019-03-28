cmake_minimum_required(VERSION 2.8.3)
project(ipa_door_handle_detection)

set(CMAKE_CXX_STANDARD 11)

## catkin
set(catkin_RUN_PACKAGES
	eigen_conversions
	pcl_conversions
	pcl_ros
	roscpp
	sensor_msgs
	tf
	tf_conversions
        cv_bridge
        image_transport
)
set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)


## System dependencies are found with CMake's conventions

# Boost
find_package(Boost REQUIRED COMPONENTS system)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS
		ros/include
	LIBRARIES
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	ros/include
	${catkin_INCLUDE_DIRS}
    	${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# door handle detection main
add_executable(ipa_door_handle_detection
    ros/src/ipa_door_handle_detection_main.cpp # main function to start handle detection
    ros/src/ipa_start_detection.cpp 
    ros/src/ipa_door_handle_segmentation.cpp
    ros/src/ipa_door_handle_template_alignment.cpp
)

add_dependencies(ipa_door_handle_detection
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ipa_door_handle_detection
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)




# template geneation
add_executable(ipa_template_generation
    ros/src/ipa_template_generation.cpp
    ros/src/ipa_door_handle_segmentation.cpp
    ros/src/ipa_door_handle_template_alignment.cpp
)

add_dependencies(ipa_template_generation
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ipa_template_generation
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)




file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
