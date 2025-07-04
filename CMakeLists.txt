PROJECT(enml)
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# ROS2 make-based build pattern (following webviz style)
MESSAGE(STATUS "Compiling with ROS2")
add_compile_definitions(ROS2)

# System dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)

# Convert AMENT_PREFIX_PATH to CMAKE_PREFIX_PATH (standard ROS2 pattern)
if(DEFINED ENV{AMENT_PREFIX_PATH})
  string(REPLACE ":" ";" ament_prefix_path_list "$ENV{AMENT_PREFIX_PATH}")
  foreach(path ${ament_prefix_path_list})
    list(APPEND CMAKE_PREFIX_PATH "${path}")
  endforeach()
endif()

find_package(amrl_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# System dependencies
FIND_PACKAGE(Lua51 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)

SET(CMAKE_CXX_FLAGS "-std=c++17 -march=native -Wall -g")

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
ADD_DEFINITIONS(-DROS2)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(src/shared)
ADD_SUBDIRECTORY(src/shared)

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/config-reader/include)
INCLUDE_DIRECTORIES(${LUA_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

# ROS2 Support Library
add_library(enml_support_lib
    src/kdtree.cpp
    src/perception_2d.cpp
    src/vector_map/vector_map.cc
    src/visualization/visualization.cc
    src/visualization/gui_helpers.cc)

target_link_libraries(enml_support_lib 
    ceres 
    glog 
    gflags
    ncurses 
    popt 
    pthread 
    boost_system
    ${LUA_LIBRARIES} 
    rt 
    amrl_shared_lib
    ${BUILD_SPECIFIC_LIBRARIES}
    rclcpp::rclcpp
    amrl_msgs::amrl_msgs__rosidl_generator_cpp
    amrl_msgs::amrl_msgs__rosidl_typesupport_cpp)

# Main ENML executable
add_executable(enml
    src/non_markov_localization_main.cpp
    src/non_markov_localization.cpp)

# Link ROS2 libraries using modern target-based pattern
target_link_libraries(enml 
    enml_support_lib
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_generator_cpp
    visualization_msgs::visualization_msgs__rosidl_generator_cpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    amrl_msgs::amrl_msgs__rosidl_generator_cpp
    amrl_msgs::amrl_msgs__rosidl_typesupport_cpp
    rosbag2_cpp::rosbag2_cpp
    rosbag2_storage::rosbag2_storage
)

# Add initialization utility
add_executable(add_initialization
    src/add_initialization.cpp)

target_link_libraries(add_initialization 
    console_bridge 
    enml_support_lib
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_generator_cpp
    amrl_msgs::amrl_msgs__rosidl_generator_cpp
    amrl_msgs::amrl_msgs__rosidl_typesupport_cpp
    rosbag2_cpp::rosbag2_cpp
    rosbag2_storage::rosbag2_storage
)

# Scene render test utility
add_executable(scene_render_test
    src/scene_render_test.cc)

target_link_libraries(scene_render_test 
    enml_support_lib
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_generator_cpp
    visualization_msgs::visualization_msgs__rosidl_generator_cpp
    amrl_msgs::amrl_msgs__rosidl_generator_cpp
    amrl_msgs::amrl_msgs__rosidl_typesupport_cpp
    ament_index_cpp::ament_index_cpp
)

# Install targets (make-based pattern)
install(TARGETS enml add_initialization scene_render_test
  RUNTIME DESTINATION bin
)

install(DIRECTORY config/
  DESTINATION share/enml/config
)

install(FILES
  README.md
  DESTINATION share/enml
)

ament_package()