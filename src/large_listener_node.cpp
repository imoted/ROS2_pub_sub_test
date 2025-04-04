#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

class ListenerNode : public rclcpp::Node {
public:
  ListenerNode() : Node("listener_node") {
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

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerNode>());
  rclcpp::shutdown();
  return 0;
}
