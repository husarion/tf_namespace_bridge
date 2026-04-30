cmake_minimum_required(VERSION 3.8)
project(tf_namespace_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

# Generated parameter library for multi_tf_namespace_bridge
generate_parameter_library(multi_tf_namespace_bridge_parameters
                           src/multi_tf_namespace_bridge_parameters.yaml)

# multi_tf_namespace_bridge
add_executable(multi_tf_namespace_bridge src/multi_tf_namespace_bridge.cpp
                                         src/multi_tf_namespace_bridge_node.cpp)
target_include_directories(multi_tf_namespace_bridge PRIVATE include)
target_link_libraries(multi_tf_namespace_bridge
                      multi_tf_namespace_bridge_parameters)
ament_target_dependencies(multi_tf_namespace_bridge rclcpp tf2_msgs)

# tf_namespace_bridge
add_executable(tf_namespace_bridge src/tf_namespace_bridge.cpp
                                   src/tf_namespace_bridge_node.cpp)
target_include_directories(tf_namespace_bridge PRIVATE include)
ament_target_dependencies(tf_namespace_bridge rclcpp tf2_msgs)

install(TARGETS multi_tf_namespace_bridge tf_namespace_bridge
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_multi_tf_namespace_bridge test/test_multi_tf_namespace_bridge.cpp
    src/multi_tf_namespace_bridge.cpp)
  target_include_directories(test_multi_tf_namespace_bridge PRIVATE include)
  target_link_libraries(test_multi_tf_namespace_bridge
                        multi_tf_namespace_bridge_parameters)
  ament_target_dependencies(test_multi_tf_namespace_bridge rclcpp tf2_msgs)

  ament_add_gtest(test_tf_namespace_bridge test/test_tf_namespace_bridge.cpp
                  src/tf_namespace_bridge.cpp)
  target_include_directories(test_tf_namespace_bridge PRIVATE include)
  ament_target_dependencies(test_tf_namespace_bridge rclcpp tf2_msgs)
endif()

ament_package()
