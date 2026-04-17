cmake_minimum_required(VERSION 3.8)
project(tf_namespace_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

# multi_namespace_tf_bridge
add_executable(multi_namespace_tf_bridge src/multi_namespace_tf_bridge.cpp
                                         src/multi_namespace_tf_bridge_node.cpp)
target_include_directories(multi_namespace_tf_bridge PRIVATE include)
ament_target_dependencies(multi_namespace_tf_bridge rclcpp tf2_msgs)

# namespace_tf_bridge
add_executable(namespace_tf_bridge src/namespace_tf_bridge.cpp
                                   src/namespace_tf_bridge_node.cpp)
target_include_directories(namespace_tf_bridge PRIVATE include)
ament_target_dependencies(namespace_tf_bridge rclcpp tf2_msgs)

install(TARGETS multi_namespace_tf_bridge namespace_tf_bridge
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_multi_namespace_tf_bridge
                  test/test_multi_namespace_tf_bridge.cpp
                  src/multi_namespace_tf_bridge.cpp)
  target_include_directories(test_multi_namespace_tf_bridge PRIVATE include)
  ament_target_dependencies(test_multi_namespace_tf_bridge rclcpp tf2_msgs)

  ament_add_gtest(test_namespace_tf_bridge test/test_namespace_tf_bridge.cpp
                  src/namespace_tf_bridge.cpp)
  target_include_directories(test_namespace_tf_bridge PRIVATE include)
  ament_target_dependencies(test_namespace_tf_bridge rclcpp tf2_msgs)
endif()

ament_package()
