cmake_minimum_required(VERSION 3.8)
project(xsens_mvn_ros2_stream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_link_options(-Wl,--no-undefined)

find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xsens_mvn_msgs REQUIRED)
find_package(xsens_mvn_ros2_common REQUIRED)
find_package(xsens_mvn_sdk REQUIRED)

generate_parameter_library(xsens_stream_node_parameters
  src/xsens_stream_node_parameters.yaml
)

add_library(${PROJECT_NAME}_HumanDataHandler SHARED src/human_data_handler.cpp)
target_include_directories(${PROJECT_NAME}_HumanDataHandler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_HumanDataHandler
  Eigen3::Eigen
  rclcpp::rclcpp
)

add_library(${PROJECT_NAME}_XsensStreamClient SHARED
  src/xsens_stream_client.cpp
  src/socket.cpp
)
target_include_directories(${PROJECT_NAME}_XsensStreamClient PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_XsensStreamClient
  ${PROJECT_NAME}_HumanDataHandler
  xsens_mvn_ros2_common::xsens_mvn_ros2_common_SkeletonPublisher
  xsens_mvn_sdk::xsens_mvn_sdk
  Boost::chrono
  Boost::thread
  rclcpp::rclcpp
)

add_executable(xsens_mvn_ros2_stream_node src/xsens_stream_node.cpp)
target_include_directories(xsens_mvn_ros2_stream_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(xsens_mvn_ros2_stream_node
  ${PROJECT_NAME}_XsensStreamClient
  xsens_mvn_ros2_common::xsens_mvn_ros2_common_SkeletonPublisher
  xsens_stream_node_parameters
  ${diagnostic_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
)

install(TARGETS
  ${PROJECT_NAME}_HumanDataHandler
  ${PROJECT_NAME}_XsensStreamClient
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS xsens_mvn_ros2_stream_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Eigen3 rclcpp xsens_mvn_ros2_common xsens_mvn_sdk)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_human_data_handler test/test_human_data_handler.cpp)
  target_include_directories(test_human_data_handler PRIVATE include)
  target_link_libraries(test_human_data_handler ${PROJECT_NAME}_HumanDataHandler)

  ament_add_gtest(test_avatar_demux test/test_avatar_demux.cpp)
  target_include_directories(test_avatar_demux PRIVATE include)
  target_compile_definitions(test_avatar_demux PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
  )
  target_link_libraries(test_avatar_demux ${PROJECT_NAME}_XsensStreamClient)

  ament_add_gtest(test_stream_client_mock test/test_stream_client_mock.cpp)
  target_link_libraries(test_stream_client_mock
    xsens_mvn_ros2_common::xsens_mvn_ros2_common_SkeletonPublisher
  )
endif()

ament_package()
