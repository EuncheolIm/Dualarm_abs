set(GLM_VERSION "0.9.8")
set(GLM_INCLUDE_DIRS "/home/kist/euncheol/Dualarm-MPC/build/_deps/glm_lib-src")

if (NOT CMAKE_VERSION VERSION_LESS "3.0")
    include("${CMAKE_CURRENT_LIST_DIR}/glmTargets.cmake")
endif()
