#include "rclcpp/rclcpp.hpp"
#include "large_data_sub/large_listener_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<large_data_sub::ListenerComponent>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
