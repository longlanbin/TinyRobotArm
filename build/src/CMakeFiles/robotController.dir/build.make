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
include src/CMakeFiles/robotController.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/robotController.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/robotController.dir/flags.make

src/CMakeFiles/robotController.dir/startup.cpp.o: src/CMakeFiles/robotController.dir/flags.make
src/CMakeFiles/robotController.dir/startup.cpp.o: ../src/startup.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/robotController.dir/startup.cpp.o"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robotController.dir/startup.cpp.o -c /home/llb/myRobotController/src/startup.cpp

src/CMakeFiles/robotController.dir/startup.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robotController.dir/startup.cpp.i"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/src/startup.cpp > CMakeFiles/robotController.dir/startup.cpp.i

src/CMakeFiles/robotController.dir/startup.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robotController.dir/startup.cpp.s"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/src/startup.cpp -o CMakeFiles/robotController.dir/startup.cpp.s

src/CMakeFiles/robotController.dir/startup.cpp.o.requires:

.PHONY : src/CMakeFiles/robotController.dir/startup.cpp.o.requires

src/CMakeFiles/robotController.dir/startup.cpp.o.provides: src/CMakeFiles/robotController.dir/startup.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/robotController.dir/build.make src/CMakeFiles/robotController.dir/startup.cpp.o.provides.build
.PHONY : src/CMakeFiles/robotController.dir/startup.cpp.o.provides

src/CMakeFiles/robotController.dir/startup.cpp.o.provides.build: src/CMakeFiles/robotController.dir/startup.cpp.o


src/CMakeFiles/robotController.dir/General6S.cpp.o: src/CMakeFiles/robotController.dir/flags.make
src/CMakeFiles/robotController.dir/General6S.cpp.o: ../src/General6S.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/robotController.dir/General6S.cpp.o"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robotController.dir/General6S.cpp.o -c /home/llb/myRobotController/src/General6S.cpp

src/CMakeFiles/robotController.dir/General6S.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robotController.dir/General6S.cpp.i"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/src/General6S.cpp > CMakeFiles/robotController.dir/General6S.cpp.i

src/CMakeFiles/robotController.dir/General6S.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robotController.dir/General6S.cpp.s"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/src/General6S.cpp -o CMakeFiles/robotController.dir/General6S.cpp.s

src/CMakeFiles/robotController.dir/General6S.cpp.o.requires:

.PHONY : src/CMakeFiles/robotController.dir/General6S.cpp.o.requires

src/CMakeFiles/robotController.dir/General6S.cpp.o.provides: src/CMakeFiles/robotController.dir/General6S.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/robotController.dir/build.make src/CMakeFiles/robotController.dir/General6S.cpp.o.provides.build
.PHONY : src/CMakeFiles/robotController.dir/General6S.cpp.o.provides

src/CMakeFiles/robotController.dir/General6S.cpp.o.provides.build: src/CMakeFiles/robotController.dir/General6S.cpp.o


src/CMakeFiles/robotController.dir/Scara4S.cpp.o: src/CMakeFiles/robotController.dir/flags.make
src/CMakeFiles/robotController.dir/Scara4S.cpp.o: ../src/Scara4S.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/robotController.dir/Scara4S.cpp.o"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robotController.dir/Scara4S.cpp.o -c /home/llb/myRobotController/src/Scara4S.cpp

src/CMakeFiles/robotController.dir/Scara4S.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robotController.dir/Scara4S.cpp.i"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/src/Scara4S.cpp > CMakeFiles/robotController.dir/Scara4S.cpp.i

src/CMakeFiles/robotController.dir/Scara4S.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robotController.dir/Scara4S.cpp.s"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/src/Scara4S.cpp -o CMakeFiles/robotController.dir/Scara4S.cpp.s

src/CMakeFiles/robotController.dir/Scara4S.cpp.o.requires:

.PHONY : src/CMakeFiles/robotController.dir/Scara4S.cpp.o.requires

src/CMakeFiles/robotController.dir/Scara4S.cpp.o.provides: src/CMakeFiles/robotController.dir/Scara4S.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/robotController.dir/build.make src/CMakeFiles/robotController.dir/Scara4S.cpp.o.provides.build
.PHONY : src/CMakeFiles/robotController.dir/Scara4S.cpp.o.provides

src/CMakeFiles/robotController.dir/Scara4S.cpp.o.provides.build: src/CMakeFiles/robotController.dir/Scara4S.cpp.o


src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o: src/CMakeFiles/robotController.dir/flags.make
src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o: ../src/SerialRobotModel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robotController.dir/SerialRobotModel.cpp.o -c /home/llb/myRobotController/src/SerialRobotModel.cpp

src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robotController.dir/SerialRobotModel.cpp.i"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/src/SerialRobotModel.cpp > CMakeFiles/robotController.dir/SerialRobotModel.cpp.i

src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robotController.dir/SerialRobotModel.cpp.s"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/src/SerialRobotModel.cpp -o CMakeFiles/robotController.dir/SerialRobotModel.cpp.s

src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o.requires:

.PHONY : src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o.requires

src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o.provides: src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/robotController.dir/build.make src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o.provides.build
.PHONY : src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o.provides

src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o.provides.build: src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o


src/CMakeFiles/robotController.dir/Controller.cpp.o: src/CMakeFiles/robotController.dir/flags.make
src/CMakeFiles/robotController.dir/Controller.cpp.o: ../src/Controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/robotController.dir/Controller.cpp.o"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robotController.dir/Controller.cpp.o -c /home/llb/myRobotController/src/Controller.cpp

src/CMakeFiles/robotController.dir/Controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robotController.dir/Controller.cpp.i"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/src/Controller.cpp > CMakeFiles/robotController.dir/Controller.cpp.i

src/CMakeFiles/robotController.dir/Controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robotController.dir/Controller.cpp.s"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/src/Controller.cpp -o CMakeFiles/robotController.dir/Controller.cpp.s

src/CMakeFiles/robotController.dir/Controller.cpp.o.requires:

.PHONY : src/CMakeFiles/robotController.dir/Controller.cpp.o.requires

src/CMakeFiles/robotController.dir/Controller.cpp.o.provides: src/CMakeFiles/robotController.dir/Controller.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/robotController.dir/build.make src/CMakeFiles/robotController.dir/Controller.cpp.o.provides.build
.PHONY : src/CMakeFiles/robotController.dir/Controller.cpp.o.provides

src/CMakeFiles/robotController.dir/Controller.cpp.o.provides.build: src/CMakeFiles/robotController.dir/Controller.cpp.o


src/CMakeFiles/robotController.dir/Robot.cpp.o: src/CMakeFiles/robotController.dir/flags.make
src/CMakeFiles/robotController.dir/Robot.cpp.o: ../src/Robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/robotController.dir/Robot.cpp.o"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robotController.dir/Robot.cpp.o -c /home/llb/myRobotController/src/Robot.cpp

src/CMakeFiles/robotController.dir/Robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robotController.dir/Robot.cpp.i"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/llb/myRobotController/src/Robot.cpp > CMakeFiles/robotController.dir/Robot.cpp.i

src/CMakeFiles/robotController.dir/Robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robotController.dir/Robot.cpp.s"
	cd /home/llb/myRobotController/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/llb/myRobotController/src/Robot.cpp -o CMakeFiles/robotController.dir/Robot.cpp.s

src/CMakeFiles/robotController.dir/Robot.cpp.o.requires:

.PHONY : src/CMakeFiles/robotController.dir/Robot.cpp.o.requires

src/CMakeFiles/robotController.dir/Robot.cpp.o.provides: src/CMakeFiles/robotController.dir/Robot.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/robotController.dir/build.make src/CMakeFiles/robotController.dir/Robot.cpp.o.provides.build
.PHONY : src/CMakeFiles/robotController.dir/Robot.cpp.o.provides

src/CMakeFiles/robotController.dir/Robot.cpp.o.provides.build: src/CMakeFiles/robotController.dir/Robot.cpp.o


# Object files for target robotController
robotController_OBJECTS = \
"CMakeFiles/robotController.dir/startup.cpp.o" \
"CMakeFiles/robotController.dir/General6S.cpp.o" \
"CMakeFiles/robotController.dir/Scara4S.cpp.o" \
"CMakeFiles/robotController.dir/SerialRobotModel.cpp.o" \
"CMakeFiles/robotController.dir/Controller.cpp.o" \
"CMakeFiles/robotController.dir/Robot.cpp.o"

# External object files for target robotController
robotController_EXTERNAL_OBJECTS =

bin/robotController: src/CMakeFiles/robotController.dir/startup.cpp.o
bin/robotController: src/CMakeFiles/robotController.dir/General6S.cpp.o
bin/robotController: src/CMakeFiles/robotController.dir/Scara4S.cpp.o
bin/robotController: src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o
bin/robotController: src/CMakeFiles/robotController.dir/Controller.cpp.o
bin/robotController: src/CMakeFiles/robotController.dir/Robot.cpp.o
bin/robotController: src/CMakeFiles/robotController.dir/build.make
bin/robotController: lib/libinterpolation.so
bin/robotController: src/CMakeFiles/robotController.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/llb/myRobotController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable ../bin/robotController"
	cd /home/llb/myRobotController/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robotController.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/robotController.dir/build: bin/robotController

.PHONY : src/CMakeFiles/robotController.dir/build

src/CMakeFiles/robotController.dir/requires: src/CMakeFiles/robotController.dir/startup.cpp.o.requires
src/CMakeFiles/robotController.dir/requires: src/CMakeFiles/robotController.dir/General6S.cpp.o.requires
src/CMakeFiles/robotController.dir/requires: src/CMakeFiles/robotController.dir/Scara4S.cpp.o.requires
src/CMakeFiles/robotController.dir/requires: src/CMakeFiles/robotController.dir/SerialRobotModel.cpp.o.requires
src/CMakeFiles/robotController.dir/requires: src/CMakeFiles/robotController.dir/Controller.cpp.o.requires
src/CMakeFiles/robotController.dir/requires: src/CMakeFiles/robotController.dir/Robot.cpp.o.requires

.PHONY : src/CMakeFiles/robotController.dir/requires

src/CMakeFiles/robotController.dir/clean:
	cd /home/llb/myRobotController/build/src && $(CMAKE_COMMAND) -P CMakeFiles/robotController.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/robotController.dir/clean

src/CMakeFiles/robotController.dir/depend:
	cd /home/llb/myRobotController/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/llb/myRobotController /home/llb/myRobotController/src /home/llb/myRobotController/build /home/llb/myRobotController/build/src /home/llb/myRobotController/build/src/CMakeFiles/robotController.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/robotController.dir/depend

