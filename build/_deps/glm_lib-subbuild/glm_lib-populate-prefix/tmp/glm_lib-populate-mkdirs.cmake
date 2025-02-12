# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-src"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-build"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-subbuild/glm_lib-populate-prefix"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-subbuild/glm_lib-populate-prefix/tmp"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-subbuild/glm_lib-populate-prefix/src/glm_lib-populate-stamp"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-subbuild/glm_lib-populate-prefix/src"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-subbuild/glm_lib-populate-prefix/src/glm_lib-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-subbuild/glm_lib-populate-prefix/src/glm_lib-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-subbuild/glm_lib-populate-prefix/src/glm_lib-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
