# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kist/euncheol/Dualarm-MPC

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kist/euncheol/Dualarm-MPC/build

# Include any dependencies generated for this target.
include _deps/qhull-build/CMakeFiles/qhalf.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include _deps/qhull-build/CMakeFiles/qhalf.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/qhull-build/CMakeFiles/qhalf.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/qhull-build/CMakeFiles/qhalf.dir/flags.make

_deps/qhull-build/CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.o: _deps/qhull-build/CMakeFiles/qhalf.dir/flags.make
_deps/qhull-build/CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.o: _deps/qhull-src/src/qhalf/qhalf.c
_deps/qhull-build/CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.o: _deps/qhull-build/CMakeFiles/qhalf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object _deps/qhull-build/CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.o"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT _deps/qhull-build/CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.o -MF CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.o.d -o CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.o -c /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-src/src/qhalf/qhalf.c

_deps/qhull-build/CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.i"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-src/src/qhalf/qhalf.c > CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.i

_deps/qhull-build/CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.s"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-build && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-src/src/qhalf/qhalf.c -o CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.s

# Object files for target qhalf
qhalf_OBJECTS = \
"CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.o"

# External object files for target qhalf
qhalf_EXTERNAL_OBJECTS =

bin/qhalf: _deps/qhull-build/CMakeFiles/qhalf.dir/src/qhalf/qhalf.c.o
bin/qhalf: _deps/qhull-build/CMakeFiles/qhalf.dir/build.make
bin/qhalf: lib/libqhullstatic.a
bin/qhalf: _deps/qhull-build/CMakeFiles/qhalf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable ../../bin/qhalf"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qhalf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/qhull-build/CMakeFiles/qhalf.dir/build: bin/qhalf
.PHONY : _deps/qhull-build/CMakeFiles/qhalf.dir/build

_deps/qhull-build/CMakeFiles/qhalf.dir/clean:
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-build && $(CMAKE_COMMAND) -P CMakeFiles/qhalf.dir/cmake_clean.cmake
.PHONY : _deps/qhull-build/CMakeFiles/qhalf.dir/clean

_deps/qhull-build/CMakeFiles/qhalf.dir/depend:
	cd /home/kist/euncheol/Dualarm-MPC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kist/euncheol/Dualarm-MPC /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-src /home/kist/euncheol/Dualarm-MPC/build /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-build /home/kist/euncheol/Dualarm-MPC/build/_deps/qhull-build/CMakeFiles/qhalf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/qhull-build/CMakeFiles/qhalf.dir/depend

