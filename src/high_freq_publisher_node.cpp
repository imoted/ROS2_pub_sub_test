#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HighFreqPublisher : public rclcpp::Node
{
public:
  HighFreqPublisher() : Node("high_freq_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("high_freq_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(10), // 100kHz = 10μs
      std::bind(&HighFreqPublisher::publish_message, this));
  }

private:
  void publish_message()
  {
    static int cnt = 0;
    ++cnt;
    auto message = std_msgs::msg::String();
    message.data = "hello world " + std::to_string(cnt);
    RCLCPP_INFO(this->get_logger(), "パブリッシュ: %s, 送信回数: %d", message.data.c_str(), cnt);
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HighFreqPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
