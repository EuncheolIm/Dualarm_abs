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

# Utility rule file for copyShaders.

# Include any custom commands dependencies for this target.
include _deps/sdflib-build/CMakeFiles/copyShaders.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/sdflib-build/CMakeFiles/copyShaders.dir/progress.make

_deps/sdflib-build/basic.frag: _deps/sdflib-src/src/render_engine/shaders/basic.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating basic.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/basic.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/basic.frag

_deps/sdflib-build/basic.vert: _deps/sdflib-src/src/render_engine/shaders/basic.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating basic.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/basic.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/basic.vert

_deps/sdflib-build/colors.frag: _deps/sdflib-src/src/render_engine/shaders/colors.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating colors.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/colors.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/colors.frag

_deps/sdflib-build/colors.vert: _deps/sdflib-src/src/render_engine/shaders/colors.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating colors.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/colors.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/colors.vert

_deps/sdflib-build/grid.frag: _deps/sdflib-src/src/render_engine/shaders/grid.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating grid.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/grid.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/grid.frag

_deps/sdflib-build/grid.vert: _deps/sdflib-src/src/render_engine/shaders/grid.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating grid.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/grid.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/grid.vert

_deps/sdflib-build/normals.frag: _deps/sdflib-src/src/render_engine/shaders/normals.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating normals.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/normals.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/normals.frag

_deps/sdflib-build/normals.vert: _deps/sdflib-src/src/render_engine/shaders/normals.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating normals.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/normals.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/normals.vert

_deps/sdflib-build/normalsSplitPlane.frag: _deps/sdflib-src/src/render_engine/shaders/normalsSplitPlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Generating normalsSplitPlane.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/normalsSplitPlane.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/normalsSplitPlane.frag

_deps/sdflib-build/normalsSplitPlane.vert: _deps/sdflib-src/src/render_engine/shaders/normalsSplitPlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Generating normalsSplitPlane.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/normalsSplitPlane.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/normalsSplitPlane.vert

_deps/sdflib-build/screenPlane.frag: _deps/sdflib-src/src/render_engine/shaders/screenPlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Generating screenPlane.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/screenPlane.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/screenPlane.frag

_deps/sdflib-build/screenPlane.vert: _deps/sdflib-src/src/render_engine/shaders/screenPlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Generating screenPlane.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/screenPlane.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/screenPlane.vert

_deps/sdflib-build/sdfOctreeLight.frag: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeLight.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Generating sdfOctreeLight.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeLight.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/sdfOctreeLight.frag

_deps/sdflib-build/sdfOctreeLight.vert: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeLight.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Generating sdfOctreeLight.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeLight.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/sdfOctreeLight.vert

_deps/sdflib-build/sdfOctreeMeanTrianglesPlane.frag: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeMeanTrianglesPlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Generating sdfOctreeMeanTrianglesPlane.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeMeanTrianglesPlane.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/sdfOctreeMeanTrianglesPlane.frag

_deps/sdflib-build/sdfOctreeMeanTrianglesPlane.vert: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeMeanTrianglesPlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Generating sdfOctreeMeanTrianglesPlane.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeMeanTrianglesPlane.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/sdfOctreeMeanTrianglesPlane.vert

_deps/sdflib-build/sdfOctreePlane.frag: _deps/sdflib-src/src/render_engine/shaders/sdfOctreePlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Generating sdfOctreePlane.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreePlane.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/sdfOctreePlane.frag

_deps/sdflib-build/sdfOctreePlane.vert: _deps/sdflib-src/src/render_engine/shaders/sdfOctreePlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Generating sdfOctreePlane.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreePlane.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/sdfOctreePlane.vert

_deps/sdflib-build/sdfOctreeRender.comp: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeRender.comp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Generating sdfOctreeRender.comp"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeRender.comp /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/sdfOctreeRender.comp

_deps/sdflib-build/sdfPlane.frag: _deps/sdflib-src/src/render_engine/shaders/sdfPlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Generating sdfPlane.frag"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/sdfPlane.frag /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/sdfPlane.frag

_deps/sdflib-build/sdfPlane.vert: _deps/sdflib-src/src/render_engine/shaders/sdfPlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kist/euncheol/Dualarm-MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_21) "Generating sdfPlane.vert"
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && cmake -E copy_if_different /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src/src/render_engine/shaders/sdfPlane.vert /home/kist/euncheol/Dualarm-MPC/build/lib/shaders/sdfPlane.vert

copyShaders: _deps/sdflib-build/basic.frag
copyShaders: _deps/sdflib-build/basic.vert
copyShaders: _deps/sdflib-build/colors.frag
copyShaders: _deps/sdflib-build/colors.vert
copyShaders: _deps/sdflib-build/grid.frag
copyShaders: _deps/sdflib-build/grid.vert
copyShaders: _deps/sdflib-build/normals.frag
copyShaders: _deps/sdflib-build/normals.vert
copyShaders: _deps/sdflib-build/normalsSplitPlane.frag
copyShaders: _deps/sdflib-build/normalsSplitPlane.vert
copyShaders: _deps/sdflib-build/screenPlane.frag
copyShaders: _deps/sdflib-build/screenPlane.vert
copyShaders: _deps/sdflib-build/sdfOctreeLight.frag
copyShaders: _deps/sdflib-build/sdfOctreeLight.vert
copyShaders: _deps/sdflib-build/sdfOctreeMeanTrianglesPlane.frag
copyShaders: _deps/sdflib-build/sdfOctreeMeanTrianglesPlane.vert
copyShaders: _deps/sdflib-build/sdfOctreePlane.frag
copyShaders: _deps/sdflib-build/sdfOctreePlane.vert
copyShaders: _deps/sdflib-build/sdfOctreeRender.comp
copyShaders: _deps/sdflib-build/sdfPlane.frag
copyShaders: _deps/sdflib-build/sdfPlane.vert
copyShaders: _deps/sdflib-build/CMakeFiles/copyShaders.dir/build.make
.PHONY : copyShaders

# Rule to build all files generated by this target.
_deps/sdflib-build/CMakeFiles/copyShaders.dir/build: copyShaders
.PHONY : _deps/sdflib-build/CMakeFiles/copyShaders.dir/build

_deps/sdflib-build/CMakeFiles/copyShaders.dir/clean:
	cd /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build && $(CMAKE_COMMAND) -P CMakeFiles/copyShaders.dir/cmake_clean.cmake
.PHONY : _deps/sdflib-build/CMakeFiles/copyShaders.dir/clean

_deps/sdflib-build/CMakeFiles/copyShaders.dir/depend:
	cd /home/kist/euncheol/Dualarm-MPC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kist/euncheol/Dualarm-MPC /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src /home/kist/euncheol/Dualarm-MPC/build /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build /home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build/CMakeFiles/copyShaders.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/sdflib-build/CMakeFiles/copyShaders.dir/depend

