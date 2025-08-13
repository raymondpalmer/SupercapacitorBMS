#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fake_can_pub");
  auto pub  = node->create_publisher<can_msgs::msg::Frame>("/from_can", 10);

  rclcpp::WallTimer<std::chrono::milliseconds>::SharedPtr timer;
  timer = node->create_wall_timer(1000ms, [pub](){
    can_msgs::msg::Frame f;
    // 示例：0x02F4（BATT_ST1 标准帧）：48.30V, -12.4A, SOC 76%, SOH 98%, Flags 0x1D, Alive 0x1
    f.id = 0x02F4;
    f.is_extended = false;
    f.is_error = false;
    f.is_rtr = false;
    f.dlc = 8;
    // 数据：DE 12 84 FF 98 C4 1D 01
    f.data = {0xDE,0x12,0x84,0xFF,0x98,0xC4,0x1D,0x01};
    pub->publish(f);
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
