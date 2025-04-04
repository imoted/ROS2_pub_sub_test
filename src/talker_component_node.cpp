#include "large_data_pub/large_talker_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<large_data_pub::TalkerComponent>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
