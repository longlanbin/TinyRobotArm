# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/llb/myRobotController

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/llb/myRobotController/build

# Include any dependencies generated for this target.
include lib/CMakeFiles/libinterpolation.dir/depend.make

# Include the progress variables for this target.
include lib/CMakeFiles/libinterpolation.dir/progress.make

# Include the compile flags for this target's objects.
include lib/CMakeFiles/libinterpolation.dir/flags.make

lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o: lib/CMakeFiles/libinterpolation.dir/flags.make
lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o: ../lib/ArcInterp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o -c /home/llb/myRobotController/lib/ArcInterp.cpp

lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libinterpolation.dir/ArcInterp.cpp.i"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/lib/ArcInterp.cpp > CMakeFiles/libinterpolation.dir/ArcInterp.cpp.i

lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libinterpolation.dir/ArcInterp.cpp.s"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/lib/ArcInterp.cpp -o CMakeFiles/libinterpolation.dir/ArcInterp.cpp.s

lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o.requires:

.PHONY : lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o.requires

lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o.provides: lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o.requires
	$(MAKE) -f lib/CMakeFiles/libinterpolation.dir/build.make lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o.provides.build
.PHONY : lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o.provides

lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o.provides.build: lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o


lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o: lib/CMakeFiles/libinterpolation.dir/flags.make
lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o: ../lib/Interpolation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libinterpolation.dir/Interpolation.cpp.o -c /home/llb/myRobotController/lib/Interpolation.cpp

lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libinterpolation.dir/Interpolation.cpp.i"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/lib/Interpolation.cpp > CMakeFiles/libinterpolation.dir/Interpolation.cpp.i

lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libinterpolation.dir/Interpolation.cpp.s"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/lib/Interpolation.cpp -o CMakeFiles/libinterpolation.dir/Interpolation.cpp.s

lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o.requires:

.PHONY : lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o.requires

lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o.provides: lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o.requires
	$(MAKE) -f lib/CMakeFiles/libinterpolation.dir/build.make lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o.provides.build
.PHONY : lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o.provides

lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o.provides.build: lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o


lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o: lib/CMakeFiles/libinterpolation.dir/flags.make
lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o: ../lib/JointInterp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libinterpolation.dir/JointInterp.cpp.o -c /home/llb/myRobotController/lib/JointInterp.cpp

lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libinterpolation.dir/JointInterp.cpp.i"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/lib/JointInterp.cpp > CMakeFiles/libinterpolation.dir/JointInterp.cpp.i

lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libinterpolation.dir/JointInterp.cpp.s"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/lib/JointInterp.cpp -o CMakeFiles/libinterpolation.dir/JointInterp.cpp.s

lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o.requires:

.PHONY : lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o.requires

lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o.provides: lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o.requires
	$(MAKE) -f lib/CMakeFiles/libinterpolation.dir/build.make lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o.provides.build
.PHONY : lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o.provides

lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o.provides.build: lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o


lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o: lib/CMakeFiles/libinterpolation.dir/flags.make
lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o: ../lib/LineInterp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/libinterpolation.dir/LineInterp.cpp.o -c /home/llb/myRobotController/lib/LineInterp.cpp

lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/libinterpolation.dir/LineInterp.cpp.i"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/lib/LineInterp.cpp > CMakeFiles/libinterpolation.dir/LineInterp.cpp.i

lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/libinterpolation.dir/LineInterp.cpp.s"
	cd /home/llb/myRobotController/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/lib/LineInterp.cpp -o CMakeFiles/libinterpolation.dir/LineInterp.cpp.s

lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o.requires:

.PHONY : lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o.requires

lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o.provides: lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o.requires
	$(MAKE) -f lib/CMakeFiles/libinterpolation.dir/build.make lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o.provides.build
.PHONY : lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o.provides

lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o.provides.build: lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o


# Object files for target libinterpolation
libinterpolation_OBJECTS = \
"CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o" \
"CMakeFiles/libinterpolation.dir/Interpolation.cpp.o" \
"CMakeFiles/libinterpolation.dir/JointInterp.cpp.o" \
"CMakeFiles/libinterpolation.dir/LineInterp.cpp.o"

# External object files for target libinterpolation
libinterpolation_EXTERNAL_OBJECTS =

lib/libinterpolation.so: lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o
lib/libinterpolation.so: lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o
lib/libinterpolation.so: lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o
lib/libinterpolation.so: lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o
lib/libinterpolation.so: lib/CMakeFiles/libinterpolation.dir/build.make
lib/libinterpolation.so: lib/CMakeFiles/libinterpolation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared library libinterpolation.so"
	cd /home/llb/myRobotController/build/lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libinterpolation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/CMakeFiles/libinterpolation.dir/build: lib/libinterpolation.so

.PHONY : lib/CMakeFiles/libinterpolation.dir/build

lib/CMakeFiles/libinterpolation.dir/requires: lib/CMakeFiles/libinterpolation.dir/ArcInterp.cpp.o.requires
lib/CMakeFiles/libinterpolation.dir/requires: lib/CMakeFiles/libinterpolation.dir/Interpolation.cpp.o.requires
lib/CMakeFiles/libinterpolation.dir/requires: lib/CMakeFiles/libinterpolation.dir/JointInterp.cpp.o.requires
lib/CMakeFiles/libinterpolation.dir/requires: lib/CMakeFiles/libinterpolation.dir/LineInterp.cpp.o.requires

.PHONY : lib/CMakeFiles/libinterpolation.dir/requires

lib/CMakeFiles/libinterpolation.dir/clean:
	cd /home/llb/myRobotController/build/lib && $(CMAKE_COMMAND) -P CMakeFiles/libinterpolation.dir/cmake_clean.cmake
.PHONY : lib/CMakeFiles/libinterpolation.dir/clean

lib/CMakeFiles/libinterpolation.dir/depend:
	cd /home/llb/myRobotController/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/llb/myRobotController /home/llb/myRobotController/lib /home/llb/myRobotController/build /home/llb/myRobotController/build/lib /home/llb/myRobotController/build/lib/CMakeFiles/libinterpolation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/CMakeFiles/libinterpolation.dir/depend
