cmake_minimum_required(VERSION 3.1.0)

project(RGBDT)

find_package(OpenCV REQUIRED)

# Add your source files here
add_executable(rgbdt thermal_3d.cpp cv-helpers.hpp) 

# Link external libraries
target_link_libraries(rgbdt PRIVATE
    realsense2
    ${OpenCV_LIBS}
)

set_property(TARGET rgbdt PROPERTY CXX_STANDARD 11)


# Include directories for external libraries
target_include_directories(rgbdt PRIVATE
    /opt/ros/noetic/include
    /usr/include
)

# Link directories for external libraries
target_link_directories(rgbdt PRIVATE
    /usr/lib
    /opt/ros/noetic/lib
)
