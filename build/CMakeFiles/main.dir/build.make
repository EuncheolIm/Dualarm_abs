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
include CMakeFiles/main.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/main.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/main.cc.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/main.cc.o: /home/kist/euncheol/Dualarm-MPC/src/main.cc
CMakeFiles/main.dir/src/main.cc.o: CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/src/main.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/main.dir/src/main.cc.o -MF CMakeFiles/main.dir/src/main.cc.o.d -o CMakeFiles/main.dir/src/main.cc.o -c /home/kist/euncheol/Dualarm-MPC/src/main.cc

CMakeFiles/main.dir/src/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/main.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/euncheol/Dualarm-MPC/src/main.cc > CMakeFiles/main.dir/src/main.cc.i

CMakeFiles/main.dir/src/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/main.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/euncheol/Dualarm-MPC/src/main.cc -o CMakeFiles/main.dir/src/main.cc.s

CMakeFiles/main.dir/src/controller_abs.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/controller_abs.cpp.o: /home/kist/euncheol/Dualarm-MPC/src/controller_abs.cpp
CMakeFiles/main.dir/src/controller_abs.cpp.o: CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/src/controller_abs.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/main.dir/src/controller_abs.cpp.o -MF CMakeFiles/main.dir/src/controller_abs.cpp.o.d -o CMakeFiles/main.dir/src/controller_abs.cpp.o -c /home/kist/euncheol/Dualarm-MPC/src/controller_abs.cpp

CMakeFiles/main.dir/src/controller_abs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/controller_abs.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/euncheol/Dualarm-MPC/src/controller_abs.cpp > CMakeFiles/main.dir/src/controller_abs.cpp.i

CMakeFiles/main.dir/src/controller_abs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/controller_abs.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/euncheol/Dualarm-MPC/src/controller_abs.cpp -o CMakeFiles/main.dir/src/controller_abs.cpp.s

CMakeFiles/main.dir/src/trajectory.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/trajectory.cpp.o: /home/kist/euncheol/Dualarm-MPC/src/trajectory.cpp
CMakeFiles/main.dir/src/trajectory.cpp.o: CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/src/trajectory.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/main.dir/src/trajectory.cpp.o -MF CMakeFiles/main.dir/src/trajectory.cpp.o.d -o CMakeFiles/main.dir/src/trajectory.cpp.o -c /home/kist/euncheol/Dualarm-MPC/src/trajectory.cpp

CMakeFiles/main.dir/src/trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/trajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/euncheol/Dualarm-MPC/src/trajectory.cpp > CMakeFiles/main.dir/src/trajectory.cpp.i

CMakeFiles/main.dir/src/trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/trajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/euncheol/Dualarm-MPC/src/trajectory.cpp -o CMakeFiles/main.dir/src/trajectory.cpp.s

CMakeFiles/main.dir/src/quadraticprogram.cc.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/quadraticprogram.cc.o: /home/kist/euncheol/Dualarm-MPC/src/quadraticprogram.cc
CMakeFiles/main.dir/src/quadraticprogram.cc.o: CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/main.dir/src/quadraticprogram.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/main.dir/src/quadraticprogram.cc.o -MF CMakeFiles/main.dir/src/quadraticprogram.cc.o.d -o CMakeFiles/main.dir/src/quadraticprogram.cc.o -c /home/kist/euncheol/Dualarm-MPC/src/quadraticprogram.cc

CMakeFiles/main.dir/src/quadraticprogram.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/quadraticprogram.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/euncheol/Dualarm-MPC/src/quadraticprogram.cc > CMakeFiles/main.dir/src/quadraticprogram.cc.i

CMakeFiles/main.dir/src/quadraticprogram.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/quadraticprogram.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/euncheol/Dualarm-MPC/src/quadraticprogram.cc -o CMakeFiles/main.dir/src/quadraticprogram.cc.s

CMakeFiles/main.dir/src/robotmodel.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/robotmodel.cpp.o: /home/kist/euncheol/Dualarm-MPC/src/robotmodel.cpp
CMakeFiles/main.dir/src/robotmodel.cpp.o: CMakeFiles/main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/main.dir/src/robotmodel.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/main.dir/src/robotmodel.cpp.o -MF CMakeFiles/main.dir/src/robotmodel.cpp.o.d -o CMakeFiles/main.dir/src/robotmodel.cpp.o -c /home/kist/euncheol/Dualarm-MPC/src/robotmodel.cpp

CMakeFiles/main.dir/src/robotmodel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/robotmodel.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kist/euncheol/Dualarm-MPC/src/robotmodel.cpp > CMakeFiles/main.dir/src/robotmodel.cpp.i

CMakeFiles/main.dir/src/robotmodel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/robotmodel.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kist/euncheol/Dualarm-MPC/src/robotmodel.cpp -o CMakeFiles/main.dir/src/robotmodel.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/main.cc.o" \
"CMakeFiles/main.dir/src/controller_abs.cpp.o" \
"CMakeFiles/main.dir/src/trajectory.cpp.o" \
"CMakeFiles/main.dir/src/quadraticprogram.cc.o" \
"CMakeFiles/main.dir/src/robotmodel.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/src/main.cc.o
main: CMakeFiles/main.dir/src/controller_abs.cpp.o
main: CMakeFiles/main.dir/src/trajectory.cpp.o
main: CMakeFiles/main.dir/src/quadraticprogram.cc.o
main: CMakeFiles/main.dir/src/robotmodel.cpp.o
main: CMakeFiles/main.dir/build.make
main: lib/libsimulate.a
main: lib/libglfw3.a
main: lib/liblodepng.a
main: libs/qpOASES/libs/libqpOASES.so.3.2
main: libs/yaml-cpp/libyaml-cpp.so.0.8.0
main: lib/libmujoco.so.3.2.3
main: /usr/lib/x86_64-linux-gnu/librt.so
main: /usr/lib/x86_64-linux-gnu/libm.so
main: /usr/lib/x86_64-linux-gnu/libX11.so
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main
.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/kist/euncheol/Dualarm-MPC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kist/euncheol/Dualarm-MPC /home/kist/euncheol/Dualarm-MPC /home/kist/euncheol/Dualarm-MPC/build /home/kist/euncheol/Dualarm-MPC/build /home/kist/euncheol/Dualarm-MPC/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

