#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HighFreqSubscriber : public rclcpp::Node
{
public:
  HighFreqSubscriber() : Node("high_freq_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "high_freq_topic", 10,
      std::bind(&HighFreqSubscriber::message_callback, this, std::placeholders::_1));
  }

private:
  void message_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    static int count = 0;
    RCLCPP_INFO(this->get_logger(), "受信: %s, 受信回数: %d", msg->data.c_str(), ++count);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HighFreqSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
