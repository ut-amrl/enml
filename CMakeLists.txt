PROJECT(enml)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(arch ${})

SET(CMAKE_CXX_FLAGS "-march=native -Wall -g")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -fno-builtin-malloc \
      -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free -O3 -DNDEBUG")
  SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

ADD_DEFINITIONS(-Wfatal-errors)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

FIND_PACKAGE(Lua51 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)

INCLUDE_DIRECTORIES(src/shared)
ADD_SUBDIRECTORY(src/shared)

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/config-reader/include)
INCLUDE_DIRECTORIES(${LUA_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

SET(libs roslib roscpp ncurses popt pthread glut boost_system
    ${LUA_LIBRARIES} rt glog rosbag amrl_shared_lib
    ${BUILD_SPECIFIC_LIBRARIES})

ROSBUILD_ADD_LIBRARY(enml_support_lib
    src/kdtree.cpp
    src/perception_2d.cpp
    src/vector_map/vector_map.cc
    src/visualization/visualization.cc
    src/visualization/gui_helpers.cc)
TARGET_LINK_LIBRARIES(enml_support_lib ceres ${libs})

ROSBUILD_ADD_EXECUTABLE(enml
    src/non_markov_localization_main.cpp
    src/non_markov_localization.cpp)
ADD_DEPENDENCIES(enml
    rosbuild_precompile
    ceres)
TARGET_LINK_LIBRARIES(enml enml_support_lib)

ROSBUILD_ADD_EXECUTABLE(add_initialization
    src/add_initialization.cpp)
TARGET_LINK_LIBRARIES(add_initialization console_bridge enml_support_lib)