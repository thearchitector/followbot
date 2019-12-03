cmake_minimum_required(VERSION 2.8.3)
project(followbot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(OpenCV_DIR /usr/local/lib)

## Find catkin macros and libraries
find_package(OpenCV 4.1.2 REQUIRED PATHS /usr/local NO_DEFAULT_PATH)
find_package(VTK 8.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
		roscpp
		geometry_msgs
		std_msgs
		message_generation
)


add_message_files(
		FILES
		Point2.msg
)

generate_messages(
		DEPENDENCIES
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS
			roscpp
			message_generation
			message_runtime
			std_msgs
			geometry_msgs
		DEPENDS OpenCV
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
sudo )

## Link the executable to external libraries
link_directories(/usr/local/lib)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(loc_and_pc src/loc_and_pc.cpp src/cloud.cpp src/human.cpp)
target_link_libraries(loc_and_pc ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})