# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/emre/Desktop/rgbdt/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/emre/Desktop/rgbdt/src/build

# Include any dependencies generated for this target.
include CMakeFiles/rgbdt.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rgbdt.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rgbdt.dir/flags.make

CMakeFiles/rgbdt.dir/thermal_3d.cpp.o: CMakeFiles/rgbdt.dir/flags.make
CMakeFiles/rgbdt.dir/thermal_3d.cpp.o: ../thermal_3d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/emre/Desktop/rgbdt/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rgbdt.dir/thermal_3d.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rgbdt.dir/thermal_3d.cpp.o -c /home/emre/Desktop/rgbdt/src/thermal_3d.cpp

CMakeFiles/rgbdt.dir/thermal_3d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rgbdt.dir/thermal_3d.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/emre/Desktop/rgbdt/src/thermal_3d.cpp > CMakeFiles/rgbdt.dir/thermal_3d.cpp.i

CMakeFiles/rgbdt.dir/thermal_3d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rgbdt.dir/thermal_3d.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/emre/Desktop/rgbdt/src/thermal_3d.cpp -o CMakeFiles/rgbdt.dir/thermal_3d.cpp.s

# Object files for target rgbdt
rgbdt_OBJECTS = \
"CMakeFiles/rgbdt.dir/thermal_3d.cpp.o"

# External object files for target rgbdt
rgbdt_EXTERNAL_OBJECTS =

rgbdt: CMakeFiles/rgbdt.dir/thermal_3d.cpp.o
rgbdt: CMakeFiles/rgbdt.dir/build.make
rgbdt: /usr/local/lib/libopencv_gapi.so.4.9.0
rgbdt: /usr/local/lib/libopencv_highgui.so.4.9.0
rgbdt: /usr/local/lib/libopencv_ml.so.4.9.0
rgbdt: /usr/local/lib/libopencv_objdetect.so.4.9.0
rgbdt: /usr/local/lib/libopencv_photo.so.4.9.0
rgbdt: /usr/local/lib/libopencv_stitching.so.4.9.0
rgbdt: /usr/local/lib/libopencv_video.so.4.9.0
rgbdt: /usr/local/lib/libopencv_videoio.so.4.9.0
rgbdt: /usr/local/lib/libopencv_imgcodecs.so.4.9.0
rgbdt: /usr/local/lib/libopencv_dnn.so.4.9.0
rgbdt: /usr/local/lib/libopencv_calib3d.so.4.9.0
rgbdt: /usr/local/lib/libopencv_features2d.so.4.9.0
rgbdt: /usr/local/lib/libopencv_flann.so.4.9.0
rgbdt: /usr/local/lib/libopencv_imgproc.so.4.9.0
rgbdt: /usr/local/lib/libopencv_core.so.4.9.0
rgbdt: CMakeFiles/rgbdt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/emre/Desktop/rgbdt/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rgbdt"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rgbdt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rgbdt.dir/build: rgbdt

.PHONY : CMakeFiles/rgbdt.dir/build

CMakeFiles/rgbdt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rgbdt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rgbdt.dir/clean

CMakeFiles/rgbdt.dir/depend:
	cd /home/emre/Desktop/rgbdt/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/emre/Desktop/rgbdt/src /home/emre/Desktop/rgbdt/src /home/emre/Desktop/rgbdt/src/build /home/emre/Desktop/rgbdt/src/build /home/emre/Desktop/rgbdt/src/build/CMakeFiles/rgbdt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rgbdt.dir/depend

