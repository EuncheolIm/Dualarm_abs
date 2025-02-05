# Install script for directory: /home/kist/euncheol/Dualarm-MPC/libs/mujoco

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/plugin/elasticity/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/plugin/actuator/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/plugin/sensor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/plugin/sdf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/src/engine/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/src/user/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/src/xml/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/src/render/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/src/thread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/src/ui/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/model/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/simulate/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "runtime" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so.3.2.3" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so.3.2.3")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so.3.2.3"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/kist/euncheol/Dualarm-MPC/build/lib/libmujoco.so.3.2.3")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so.3.2.3" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so.3.2.3")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so.3.2.3")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "runtime" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/kist/euncheol/Dualarm-MPC/build/lib/libmujoco.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mujoco" TYPE FILE FILES
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjdata.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjexport.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjmacro.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjmodel.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjplugin.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjrender.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjspec.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjthread.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjtnum.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjui.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjvisualize.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mjxmacro.h"
    "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/include/mujoco/mujoco.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco/mujocoTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco/mujocoTargets.cmake"
         "/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/CMakeFiles/Export/0b3b73e44e0b7c8937151edebfc316a5/mujocoTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco/mujocoTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco/mujocoTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco" TYPE FILE FILES "/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/CMakeFiles/Export/0b3b73e44e0b7c8937151edebfc316a5/mujocoTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco" TYPE FILE FILES "/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/CMakeFiles/Export/0b3b73e44e0b7c8937151edebfc316a5/mujocoTargets-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco" TYPE FILE FILES
    "/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/mujocoConfig.cmake"
    "/home/kist/euncheol/Dualarm-MPC/build/libs/mujoco/mujocoConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mujoco" TYPE DIRECTORY FILES "/home/kist/euncheol/Dualarm-MPC/libs/mujoco/model" REGEX "/CMakeLists\\.txt$" EXCLUDE)
endif()

