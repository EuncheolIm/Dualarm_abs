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
CMAKE_SOURCE_DIR = /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild

# Utility rule file for lodepng-populate.

# Include any custom commands dependencies for this target.
include CMakeFiles/lodepng-populate.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/lodepng-populate.dir/progress.make

CMakeFiles/lodepng-populate: CMakeFiles/lodepng-populate-complete

CMakeFiles/lodepng-populate-complete: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-install
CMakeFiles/lodepng-populate-complete: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-mkdir
CMakeFiles/lodepng-populate-complete: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-download
CMakeFiles/lodepng-populate-complete: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-update
CMakeFiles/lodepng-populate-complete: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-patch
CMakeFiles/lodepng-populate-complete: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-configure
CMakeFiles/lodepng-populate-complete: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-build
CMakeFiles/lodepng-populate-complete: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-install
CMakeFiles/lodepng-populate-complete: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'lodepng-populate'"
	/home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E make_directory /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles
	/home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E touch /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles/lodepng-populate-complete
	/home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E touch /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-done

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-update:
.PHONY : lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-update

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-build: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No build step for 'lodepng-populate'"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-build && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E echo_append
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-build && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E touch /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-build

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-configure: lodepng-populate-prefix/tmp/lodepng-populate-cfgcmd.txt
lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-configure: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "No configure step for 'lodepng-populate'"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-build && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E echo_append
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-build && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E touch /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-configure

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-download: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-gitinfo.txt
lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-download: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'lodepng-populate'"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -P /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/tmp/lodepng-populate-gitclone.cmake
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E touch /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-download

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-install: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'lodepng-populate'"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-build && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E echo_append
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-build && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E touch /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-install

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'lodepng-populate'"
	/home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -Dcfgdir= -P /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/tmp/lodepng-populate-mkdirs.cmake
	/home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E touch /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-mkdir

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-patch: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'lodepng-populate'"
	/home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E echo_append
	/home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E touch /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-patch

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-update:
.PHONY : lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-update

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-test: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No test step for 'lodepng-populate'"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-build && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E echo_append
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-build && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -E touch /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-test

lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-update: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Performing update step for 'lodepng-populate'"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-src && /home/kist/anaconda3/lib/python3.9/site-packages/cmake/data/bin/cmake -P /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/lodepng-populate-prefix/tmp/lodepng-populate-gitupdate.cmake

lodepng-populate: CMakeFiles/lodepng-populate
lodepng-populate: CMakeFiles/lodepng-populate-complete
lodepng-populate: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-build
lodepng-populate: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-configure
lodepng-populate: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-download
lodepng-populate: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-install
lodepng-populate: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-mkdir
lodepng-populate: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-patch
lodepng-populate: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-test
lodepng-populate: lodepng-populate-prefix/src/lodepng-populate-stamp/lodepng-populate-update
lodepng-populate: CMakeFiles/lodepng-populate.dir/build.make
.PHONY : lodepng-populate

# Rule to build all files generated by this target.
CMakeFiles/lodepng-populate.dir/build: lodepng-populate
.PHONY : CMakeFiles/lodepng-populate.dir/build

CMakeFiles/lodepng-populate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lodepng-populate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lodepng-populate.dir/clean

CMakeFiles/lodepng-populate.dir/depend:
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild /home/kist/euncheol/Dualarm-MPC/build/_deps/lodepng-subbuild/CMakeFiles/lodepng-populate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lodepng-populate.dir/depend

