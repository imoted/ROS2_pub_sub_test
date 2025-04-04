#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class DummyLaserScanPublisher : public rclcpp::Node
{
public:
  DummyLaserScanPublisher() : Node("dummy_laser_scan_publisher")
  {
    // パブリッシャーの初期化
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    // タイマーを設定して10Hzでコールバックを実行
    timer_ =
      this->create_wall_timer(100ms, std::bind(&DummyLaserScanPublisher::timer_callback, this));

    // LaserScanメッセージの固定パラメータを設定
    scan_msg_.header.frame_id = "laser_frame";
    scan_msg_.angle_min = -M_PI;
    scan_msg_.angle_max = M_PI;
    scan_msg_.angle_increment = 0.01;  // 約0.57度
    scan_msg_.time_increment = 0.0;
    scan_msg_.scan_time = 0.1;  // 10Hzの逆数
    scan_msg_.range_min = 0.1;
    scan_msg_.range_max = 10.0;

    // ダミーデータのポイント数
    int num_points =
      static_cast<int>((scan_msg_.angle_max - scan_msg_.angle_min) / scan_msg_.angle_increment);

    // ダミーの距離データを生成
    scan_msg_.ranges.resize(num_points);
    scan_msg_.intensities.resize(num_points);

    RCLCPP_INFO(
      this->get_logger(),
      "ダミーLaserScanパブリッシャーを開始します。トピック: scan、周波数: 10Hz");
  }

private:
  void timer_callback()
  {
    // 現在時刻を設定
    scan_msg_.header.stamp = this->now();

    // ダミーの距離データを更新（5mの壁を模倣）
    for (size_t i = 0; i < scan_msg_.ranges.size(); ++i) {
      // ランダムなノイズを加えた5mの距離
      scan_msg_.ranges[i] = 5.0 + (static_cast<double>(rand()) / RAND_MAX - 0.5) * 0.1;
      scan_msg_.intensities[i] = 100.0;
    }

    // LaserScanメッセージをパブリッシュ
    publisher_->publish(scan_msg_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  sensor_msgs::msg::LaserScan scan_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyLaserScanPublisher>());
  rclcpp::shutdown();
  return 0;
}
