#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace large_data_sub {

class ListenerComponent : public rclcpp::Node {
public:
  explicit ListenerComponent(const rclcpp::NodeOptions & options)
  : Node("listener_node", options) {
    subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "large_data", 10,
      [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        static int count = 0;
        RCLCPP_INFO(this->get_logger(), "Received data of size: %zu bytes, 受信回数: %d", msg->data.size(), ++count);
      }
    );
  }

private:
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
};

}  // namespace large_data_sub

RCLCPP_COMPONENTS_REGISTER_NODE(large_data_sub::ListenerComponent)
