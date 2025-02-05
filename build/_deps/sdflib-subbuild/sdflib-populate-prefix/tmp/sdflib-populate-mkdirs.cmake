# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-src"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-build"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-subbuild/sdflib-populate-prefix"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-subbuild/sdflib-populate-prefix/tmp"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-subbuild/sdflib-populate-prefix/src/sdflib-populate-stamp"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-subbuild/sdflib-populate-prefix/src"
  "/home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-subbuild/sdflib-populate-prefix/src/sdflib-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-subbuild/sdflib-populate-prefix/src/sdflib-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/kist/euncheol/Dualarm-MPC/build/_deps/sdflib-subbuild/sdflib-populate-prefix/src/sdflib-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
