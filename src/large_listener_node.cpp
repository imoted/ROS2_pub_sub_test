#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

class ListenerNode : public rclcpp::Node
{
public:
  ListenerNode() : Node("listener_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "large_data", 10, [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        static int count = 0;

        // メッセージから送信時のタイムスタンプを取得
        int64_t send_time;
        std::memcpy(&send_time, msg->data.data(), sizeof(int64_t));

        // 現在時刻を取得して差分を計算
        int64_t now = this->now().nanoseconds();
        double latency_ms = (now - send_time) / 1000000.0;  // ナノ秒からミリ秒へ変換

        RCLCPP_INFO(
          this->get_logger(),
          "受信データサイズ: %zu バイト, 受信回数: %d, 通信時間: %.2f ms",
          msg->data.size(), ++count, latency_ms);
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerNode>());
  rclcpp::shutdown();
  return 0;
}
