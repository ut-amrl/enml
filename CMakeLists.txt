cmake_minimum_required(VERSION 3.16)
project(enml)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_definitions(ROS2)
add_compile_options(-march=native -Wall -Wfatal-errors)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE MATCHES "Release")
  message(STATUS "Additional Flags for Release mode")
  add_compile_options(-fopenmp -fno-builtin-malloc -fno-builtin-calloc
      -fno-builtin-realloc -fno-builtin-free -O3 -DNDEBUG)
  set(BUILD_SPECIFIC_LIBRARIES tcmalloc)
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
  message(STATUS "Additional Flags for Debug mode")
  add_compile_options(-g)
  set(BUILD_SPECIFIC_LIBRARIES "")
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

if(DEFINED ENV{AMENT_PREFIX_PATH})
  string(REPLACE ":" ";" AMENT_PREFIX_PATH_LIST "$ENV{AMENT_PREFIX_PATH}")
  list(APPEND CMAKE_PREFIX_PATH ${AMENT_PREFIX_PATH_LIST})
endif()

find_package(amrl_msgs REQUIRED)

find_package(Lua 5.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/shared
    ${PROJECT_SOURCE_DIR}/src/config-reader/include
    ${LUA_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)

add_subdirectory(src/shared)

add_library(enml_support_lib STATIC
    src/kdtree.cpp
    src/perception_2d.cpp
    src/vector_map/vector_map.cc
    src/visualization/visualization.cc
    src/visualization/gui_helpers.cc)
ament_target_dependencies(enml_support_lib
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    amrl_msgs)
target_link_libraries(enml_support_lib
    amrl_shared_lib
    ${CERES_LIBRARIES}
    ${LUA_LIBRARIES}
    glog
    pthread
    ${BUILD_SPECIFIC_LIBRARIES})

add_executable(enml
    src/non_markov_localization_main.cpp
    src/non_markov_localization.cpp)
ament_target_dependencies(enml
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    rosbag2_cpp
    rosbag2_storage
    ament_index_cpp
    amrl_msgs)
target_link_libraries(enml
    enml_support_lib
    OpenMP::OpenMP_CXX
    popt
    ncurses
    gflags
    glog
    pthread
    rt
    boost_system
    ${BUILD_SPECIFIC_LIBRARIES})

add_executable(add_initialization
    src/add_initialization.cpp)
ament_target_dependencies(add_initialization
    rclcpp
    geometry_msgs
    rosbag2_cpp
    rosbag2_storage)
target_link_libraries(add_initialization
    enml_support_lib
    gflags
    glog
    pthread)

add_executable(scene_render_test
    src/scene_render_test.cc)
ament_target_dependencies(scene_render_test
    rclcpp
    amrl_msgs
    visualization_msgs
    ament_index_cpp)
target_link_libraries(scene_render_test
    enml_support_lib
    gflags
    glog
    pthread)

install(TARGETS enml add_initialization scene_render_test
    RUNTIME DESTINATION bin)

ament_package()