#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace large_data_pub {

class TalkerComponent : public rclcpp::Node {
public:
  explicit TalkerComponent(const rclcpp::NodeOptions & options)
  : Node("talker_node", options) {
    publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("large_data", 10);

    message_.data.resize(1000 * 1024 * 1024, 42);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TalkerComponent::publish_data, this)
    );
  }

private:
  void publish_data() {
    static int cnt = 0;
    RCLCPP_INFO(this->get_logger(), "Publishing 1000MB data, 送信回数: %d", ++cnt);
    publisher_->publish(message_);
  }

  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::ByteMultiArray message_;
};

}  // namespace large_data_pub

RCLCPP_COMPONENTS_REGISTER_NODE(large_data_pub::TalkerComponent)
