cmake_minimum_required(VERSION 3.8)
project(xsens_mvn_ros2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_link_options(-Wl,--no-undefined)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

generate_parameter_library(xsens_urdf_publisher_parameters
  src/xsens_urdf_publisher_parameters.yaml
)

add_executable(xsens_mvn_ros2_urdf_publisher_node src/xsens_urdf_publisher_node.cpp)
target_include_directories(xsens_mvn_ros2_urdf_publisher_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(xsens_mvn_ros2_urdf_publisher_node
  xsens_urdf_publisher_parameters
  ${diagnostic_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
)

install(TARGETS xsens_mvn_ros2_urdf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_segment_defs test/test_segment_defs.cpp)
  target_include_directories(test_segment_defs PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_scale_utils test/test_scale_utils.cpp)
  target_include_directories(test_scale_utils PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
