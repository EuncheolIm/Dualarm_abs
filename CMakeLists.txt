cmake_minimum_required(VERSION 3.16)

set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
set(MSVC_INCREMENTAL_DEFAULT ON)

set(CMAKE_CXX_STANDARD 17)

project(
  template_arc
  VERSION 3.1.3
  DESCRIPTION "MuJoCo Template Repository using ARC libaray"
)

enable_language(C)
enable_language(CXX)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-no-as-needed")

message(STATUS "Eigen3 : " ${EIGEN3_INCLUDE_DIR}/Eigen)
message(STATUS "current : " ${CMAKE_CURRENT_SOURCE_DIR})

#################################
# Open Libraries ADD here
#################################
option(MUJOCO_BUILD_EXAMPLES "Build samples for MuJoCo" OFF)
option(MUJOCO_BUILD_SIMULATE "Build simulate library for MuJoCo" ON)
option(MUJOCO_BUILD_TESTS "Build tests for MuJoCo" OFF)
option(MUJOCO_TEST_PYTHON_UTIL "Build and test utility libraries for Python bindings" ON)
option(QPOASES_BUILD_EXAMPLES "Build examples." OFF)
add_subdirectory(${PROJECT_SOURCE_DIR}/libs/mujoco)                             # Add subdirectory
add_subdirectory(libs/yaml-cpp) # This is one way of referencing another CMake project
add_subdirectory(${PROJECT_SOURCE_DIR}/libs/qpOASES) # CMake Path append
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/libs/mujoco/sample/cmake") # CMake Path append

# Check if we are building as standalone project.
# set(SAMPLE_STANDALONE OFF)
# set(_INSTALL_SAMPLES ON)
# if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
#   set(SAMPLE_STANDALONE ON)
#   # If standalone, do not install the samples.
#   set(_INSTALL_SAMPLES OFF)
# endif()
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# if(SAMPLE_STANDALONE)
#   include(SampleOptions)
# else()
#   enforce_mujoco_macosx_min_version()
# endif()
# include(SampleDependencies)

# set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
# set(MUJOCO_SAMPLE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(MUJOCO_HARDEN)
  if(WIN32)
    set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -O3 -Wl,/DYNAMICBASE)
  else()
    set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${MUJOCO_SAMPLE_COMPILE_OPTIONS}" -O3 -fPIE)
    if(APPLE)
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -O3 -Wl,-pie)
    else()
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -O3 -pie)
    endif()
  endif()
endif()
add_definitions(-DCMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MUJOCO_HOME}/include
  ${MUJOCO_HOME}/simulate
  /usr/local/include/rbdl
  ${MUJOCO_HOME}/lodepng-master
  ${YAML_CPP_INCLUDE_DIR}
  DEPENDS system_lib
)

# message(STATUS "Include dir: " ${INCLUDE_DIRECTORIES})

find_package(Eigen3 REQUIRED)

include_directories(
    include
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    ${EIGEN3_INCLUDE_DIR}
    ${YAML_CPP_INCLUDE_DIR}
    DEPENDS system_lib
)
add_executable(main
    
    src/main.cc

    # src/controller.cpp
    # src/controller_j15.cpp  # MPC

    src/controller_abs.cpp  # MPC

    src/trajectory.cpp 
    src/quadraticprogram.cc
    src/robotmodel.cpp
    ${SIMULATE_RESOURCE_FILES}
)

target_include_directories(main PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/qpOASES/include
)
target_link_libraries(main
  libsimulate
  mujoco::mujoco
  glfw
  -lpthread
  lodepng
  qpOASES
  yaml-cpp
  -lrbdl
  -lrbdl_urdfreader
)
target_compile_options(main PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})

