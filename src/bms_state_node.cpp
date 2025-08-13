#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <cmath>
#include <cstdint>

using can_msgs::msg::Frame;
using sensor_msgs::msg::BatteryState;
using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

static inline uint16_t U16BE(const uint8_t *p) { return (uint16_t)p[0] << 8 | p[1]; }
static inline int16_t  S16BE(const uint8_t *p) { return (int16_t)((uint16_t)p[0] << 8 | p[1]); }

class BmsStateNode : public rclcpp::Node {
public:
  BmsStateNode() : Node("bms_state_node") {
    declare_parameter<std::string>("from_topic", "/from_can");
    declare_parameter<int>("max_cells", 128);
    declare_parameter<int>("max_temps", 64);

    auto topic = get_parameter("from_topic").as_string();
    sub_ = create_subscription<Frame>(
      topic, rclcpp::QoS(500),
      std::bind(&BmsStateNode::on_frame, this, std::placeholders::_1));

    pub_batt_  = create_publisher<BatteryState>("/bms/battery_state", 10);
    pub_power_ = create_publisher<Float32>("/bms/power", 10);
    pub_cells_ = create_publisher<Float32MultiArray>("/bms/cell_voltages", 10);
    pub_tmps_  = create_publisher<Float32MultiArray>("/bms/temps", 10);

    cells_.assign(get_parameter("max_cells").as_int(), 0.0f);
    temps_.assign(get_parameter("max_temps").as_int(), 0.0f);

    timer_ = create_wall_timer(std::chrono::milliseconds(200),
                               std::bind(&BmsStateNode::flush_arrays, this));
  }

private:
  void on_frame(const Frame &f) {
    if (f.dlc < 2) return;

    // 0x02F4：Pack Voltage/Current/SOC/SOH/Flags/Alive（与 fake_can_pub 对齐）
    if (!f.is_extended && f.id == 0x02F4 && f.dlc >= 8) {
      uint16_t v_001V = U16BE(&f.data[0]);  // 0.01V/LSB
      int16_t  i_01A  = S16BE(&f.data[2]);  // 0.1A/LSB
      uint8_t  soc    = f.data[4];          // 1%/LSB

      pack_V_  = v_001V * 0.01f;
      pack_I_  = i_01A * 0.1f;
      soc_pct_ = soc;

      publish_battery_state_();
      return;
    }

    // 0x02F5：Power（1W/LSB）
    if (!f.is_extended && f.id == 0x02F5 && f.dlc >= 2) {
      int16_t p_W = S16BE(&f.data[0]);
      power_W_ = static_cast<float>(p_W);
      publish_power_();
      return;
    }

    // 0x0300..：单体电压（每帧4节，mV，BE）
    if (!f.is_extended && f.id >= 0x0300 && f.id < 0x0310 && f.dlc == 8) {
      int base = (f.id - 0x0300) * 4;
      ensure_size_(cells_, base + 4);
      for (int i = 0; i < 4; ++i) {
        uint16_t mV = U16BE(&f.data[i * 2]);
        if (mV != 0) {
          int idx = base + i;
          if (idx < (int)cells_.size()) cells_[idx] = mV * 0.001f; // V
        }
      }
      return;
    }

    // 0x0310..：温度（每帧4个，s16 0.1℃/LSB，BE）
    if (!f.is_extended && f.id >= 0x0310 && f.id < 0x0320 && f.dlc == 8) {
      int base = (f.id - 0x0310) * 4;
      ensure_size_(temps_, base + 4);
      for (int i = 0; i < 4; ++i) {
        int16_t t01C = S16BE(&f.data[i * 2]);
        int idx = base + i;
        if (idx < (int)temps_.size()) temps_[idx] = t01C * 0.1f;
      }
      return;
    }
  }

  template <typename T>
  void ensure_size_(std::vector<T> &v, size_t n) {
    if (v.size() < n) v.resize(n, T{});
  }

  void publish_battery_state_() {
    BatteryState bs;
    bs.header.stamp = now();
    bs.voltage = pack_V_;
    bs.current = pack_I_;
    bs.percentage = soc_pct_ / 100.0f;

    if (std::abs(pack_I_) < 0.2f)
      bs.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    else if (pack_I_ > 0.0f)
      bs.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    else
      bs.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

    pub_batt_->publish(bs);

    if (power_W_ == 0.0f) {
      Float32 pw; pw.data = pack_V_ * pack_I_;
      pub_power_->publish(pw);
    }
  }

  void publish_power_() {
    Float32 pw; pw.data = power_W_;
    pub_power_->publish(pw);
  }

  void flush_arrays() {
    Float32MultiArray cv; cv.data.assign(cells_.begin(), cells_.end());
    pub_cells_->publish(cv);

    Float32MultiArray tp; tp.data.assign(temps_.begin(), temps_.end());
    pub_tmps_->publish(tp);
  }

  // members
  rclcpp::Subscription<Frame>::SharedPtr sub_;
  rclcpp::Publisher<BatteryState>::SharedPtr pub_batt_;
  rclcpp::Publisher<Float32>::SharedPtr pub_power_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_cells_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_tmps_;
  rclcpp::TimerBase::SharedPtr timer_;

  float pack_V_{0}, pack_I_{0}, power_W_{0};
  int soc_pct_{0};
  std::vector<float> cells_, temps_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BmsStateNode>());
  rclcpp::shutdown();
  return 0;
}
