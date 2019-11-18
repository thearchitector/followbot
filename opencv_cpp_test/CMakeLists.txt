cmake_minimum_required(VERSION 3.7)
project(openCVTest)
set (CMAKE_CXX_STANDARD 11)
# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(opencv_example_project)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
    # Add OpenCV headers location to your include paths
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# Declare the executable target built from your sources
add_executable(stereo_match stereo_match.cpp)
add_executable(calibrate calibrate.cpp)
add_executable(view_webcams view_webcams.cpp)
add_executable(stereo_calib stereo_calib.cpp)
add_executable(live_stereo_match live_stereo_match.cpp live_stereo_match.cpp)
add_executable(fisheye_attempt fisheye_attempt.cpp)

# Link your application with OpenCV libraries
target_link_libraries(stereo_match ${OpenCV_LIBS})
target_link_libraries(calibrate ${OpenCV_LIBS})
target_link_libraries(view_webcams ${OpenCV_LIBS})
target_link_libraries(stereo_calib ${OpenCV_LIBS})
target_link_libraries(live_stereo_match ${OpenCV_LIBS})
target_link_libraries(fisheye_attempt ${OpenCV_LIBS})
