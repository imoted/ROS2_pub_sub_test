cmake_minimum_required(VERSION 3.5)
project(pubsub_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(large_talker_node src/large_talker_node.cpp)
ament_target_dependencies(large_talker_node rclcpp std_msgs)

add_executable(large_listener_node src/large_listener_node.cpp)
ament_target_dependencies(large_listener_node rclcpp std_msgs)

add_executable(high_freq_publisher src/high_freq_publisher_node.cpp)
ament_target_dependencies(high_freq_publisher rclcpp std_msgs)

add_executable(high_freq_subscriber src/high_freq_subscriber_node.cpp)
ament_target_dependencies(high_freq_subscriber rclcpp std_msgs)

add_executable(dummy_laser_scan_publisher src/dummy_laser_scan_publisher.cpp)
ament_target_dependencies(dummy_laser_scan_publisher rclcpp sensor_msgs)

# コンポーネントライブラリ
add_library(talker_component SHARED src/large_talker_component.cpp)
ament_target_dependencies(talker_component rclcpp std_msgs rclcpp_components)

add_library(listener_component SHARED src/large_listener_component.cpp)
ament_target_dependencies(listener_component rclcpp std_msgs rclcpp_components)

# コンポーネントをエクスポート
rclcpp_components_register_nodes(talker_component "large_data_pub::TalkerComponent")
rclcpp_components_register_nodes(listener_component "large_data_sub::ListenerComponent")

# インストール
install(TARGETS
  large_talker_node
  large_listener_node
  high_freq_publisher
  high_freq_subscriber
  dummy_laser_scan_publisher
  talker_component
  listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Launchファイルをインストール
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
