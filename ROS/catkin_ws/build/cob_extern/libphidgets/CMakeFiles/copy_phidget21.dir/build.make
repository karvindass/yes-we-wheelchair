# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/wes/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wes/catkin_ws/build

# Utility rule file for copy_phidget21.

# Include the progress variables for this target.
include cob_extern/libphidgets/CMakeFiles/copy_phidget21.dir/progress.make

cob_extern/libphidgets/CMakeFiles/copy_phidget21:
	cd /home/wes/catkin_ws/build/cob_extern/libphidgets && cmake -E copy /home/wes/catkin_ws/src/cob_extern/libphidgets/lib/libphidget21.so.0 /home/wes/catkin_ws/devel/lib/libphidget21.so

copy_phidget21: cob_extern/libphidgets/CMakeFiles/copy_phidget21
copy_phidget21: cob_extern/libphidgets/CMakeFiles/copy_phidget21.dir/build.make
.PHONY : copy_phidget21

# Rule to build all files generated by this target.
cob_extern/libphidgets/CMakeFiles/copy_phidget21.dir/build: copy_phidget21
.PHONY : cob_extern/libphidgets/CMakeFiles/copy_phidget21.dir/build

cob_extern/libphidgets/CMakeFiles/copy_phidget21.dir/clean:
	cd /home/wes/catkin_ws/build/cob_extern/libphidgets && $(CMAKE_COMMAND) -P CMakeFiles/copy_phidget21.dir/cmake_clean.cmake
.PHONY : cob_extern/libphidgets/CMakeFiles/copy_phidget21.dir/clean

cob_extern/libphidgets/CMakeFiles/copy_phidget21.dir/depend:
	cd /home/wes/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wes/catkin_ws/src /home/wes/catkin_ws/src/cob_extern/libphidgets /home/wes/catkin_ws/build /home/wes/catkin_ws/build/cob_extern/libphidgets /home/wes/catkin_ws/build/cob_extern/libphidgets/CMakeFiles/copy_phidget21.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cob_extern/libphidgets/CMakeFiles/copy_phidget21.dir/depend

