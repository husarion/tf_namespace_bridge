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

# Generated parameter libraries
generate_parameter_library(multi_tf_namespace_bridge_parameters
                           src/multi_tf_namespace_bridge_parameters.yaml)
generate_parameter_library(tf_namespace_bridge_parameters
                           src/tf_namespace_bridge_parameters.yaml)

# Internal helper: glob-based frame whitelist with auto-include of missing
# parents.
add_library(frame_filter STATIC src/frame_filter.cpp)
target_include_directories(frame_filter PUBLIC include)
ament_target_dependencies(frame_filter tf2_msgs)

# multi_tf_namespace_bridge
add_executable(multi_tf_namespace_bridge src/multi_tf_namespace_bridge.cpp
                                         src/multi_tf_namespace_bridge_node.cpp)
target_include_directories(multi_tf_namespace_bridge PRIVATE include)
target_link_libraries(multi_tf_namespace_bridge frame_filter
                      multi_tf_namespace_bridge_parameters)
ament_target_dependencies(multi_tf_namespace_bridge rclcpp tf2_msgs)

# tf_namespace_bridge
add_executable(tf_namespace_bridge src/tf_namespace_bridge.cpp
                                   src/tf_namespace_bridge_node.cpp)
target_include_directories(tf_namespace_bridge PRIVATE include)
target_link_libraries(tf_namespace_bridge frame_filter
                      tf_namespace_bridge_parameters)
ament_target_dependencies(tf_namespace_bridge rclcpp tf2_msgs)

install(TARGETS multi_tf_namespace_bridge tf_namespace_bridge
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Isolate ROS 2 tests from any other process publishing on the default domain
  # (e.g. a real robot or simulator running on the same host) by using a
  # dedicated, localhost-only domain.
  set(TEST_ENV ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1)

  ament_add_gtest(
    test_multi_tf_namespace_bridge test/test_multi_tf_namespace_bridge.cpp
    src/multi_tf_namespace_bridge.cpp ENV ${TEST_ENV})
  target_include_directories(test_multi_tf_namespace_bridge PRIVATE include)
  target_link_libraries(test_multi_tf_namespace_bridge frame_filter
                        multi_tf_namespace_bridge_parameters)
  ament_target_dependencies(test_multi_tf_namespace_bridge rclcpp tf2_msgs)

  ament_add_gtest(test_tf_namespace_bridge test/test_tf_namespace_bridge.cpp
                  src/tf_namespace_bridge.cpp ENV ${TEST_ENV})
  target_include_directories(test_tf_namespace_bridge PRIVATE include)
  target_link_libraries(test_tf_namespace_bridge frame_filter
                        tf_namespace_bridge_parameters)
  ament_target_dependencies(test_tf_namespace_bridge rclcpp tf2_msgs)

  ament_add_gtest(test_frame_filter test/test_frame_filter.cpp)
  target_link_libraries(test_frame_filter frame_filter)
  ament_target_dependencies(test_frame_filter tf2_msgs)
endif()

ament_package()
