#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <chrono>
#include <random>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

static inline void pack_u16_be(uint8_t *dst, uint16_t v) { dst[0] = (v >> 8) & 0xFF; dst[1] = v & 0xFF; }
static inline void pack_s16_be(uint8_t *dst, int16_t v)  { pack_u16_be(dst, static_cast<uint16_t>(v)); }

class FakeCanPub : public rclcpp::Node {
public:
  FakeCanPub() : Node("fake_can_pub") {
    pub_ = create_publisher<can_msgs::msg::Frame>("/from_can", rclcpp::QoS(100));
    declare_parameter<int>("cells_total", 12);
    declare_parameter<int>("temps_total", 6);
    declare_parameter<double>("rate_hz", 20.0);
    // TODO: 电池容量参数，暂时注释掉，使用硬编码值
    // declare_parameter<double>("battery_capacity_ah", 10.0);

    cells_   = get_parameter("cells_total").as_int();
    temps_   = get_parameter("temps_total").as_int();
    rate_hz_ = get_parameter("rate_hz").as_double();
    // battery_capacity_ah_ = get_parameter("battery_capacity_ah").as_double();

    rng_.seed(std::random_device{}());
    cells_mV_.assign(cells_, 3950);
    temps_c_.assign(temps_, 28.0);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_)),
                               std::bind(&FakeCanPub::tick, this));
  }

private:
  void tick() {
    double t = now().seconds();

    // 电流波形 + 噪声 (A)
    current_A_ = -2.0 * (0.6 + 0.4 * std::sin(2 * M_PI * 0.03 * t)) - 0.3 * std::sin(2 * M_PI * 0.4 * t);
    current_A_ += n_(rng_) * 0.05;

    // SOC 缓慢下降
    soc_ = std::clamp(soc_ - 0.005, 5.0, 100.0);

    // 单体电压由 SOC 线性近似 + 内阻降
    double v_min = 3.0, v_max = 4.2;
    double ocv = v_min + (v_max - v_min) * (soc_ / 100.0);
    double ir  = current_A_ * 0.005; // 5 mΩ/串（简化）
    double pack_V = 0.0;
    for (int i = 0; i < cells_; ++i) {
      double v = std::clamp(ocv - ir + n_(rng_) * 0.003, v_min, v_max);
      cells_mV_[i] = (int)std::round(v * 1000.0);
      pack_V += v;
    }
    pack_mV_ = (int)std::round(pack_V * 1000.0);

    // 温度缓慢变化 + 噪声
    for (auto &tt : temps_c_) {
      tt += 0.02 * std::sin(2 * M_PI * 0.005 * t) + n_(rng_) * 0.05;
      tt = std::clamp(tt, 20.0, 60.0);
    }

    // ---- 0x02F4：总电压/电流/SOC/SOH/flags/alive ----
    {
      can_msgs::msg::Frame f;
      f.id = 0x02F4; f.is_extended = false; f.is_rtr = false; f.is_error = false; f.dlc = 8;

      uint16_t v_001V = (uint16_t)std::clamp(pack_mV_ / 10, 0, 65535);
      int16_t  i_01A  = (int16_t)std::clamp((int)std::lround(current_A_ * 10.0), -32768, 32767);

      pack_u16_be(&f.data[0], v_001V);
      pack_s16_be(&f.data[2], i_01A);
      f.data[4] = (uint8_t)std::round(soc_);
      f.data[5] = 98; // SOH 固定
      uint8_t flags = 0; if (current_A_ > 0.2) flags |= 0x01; if (current_A_ < -0.2) flags |= 0x02;
      f.data[6] = flags;
      f.data[7] = (alive_++ & 0x0F);

      pub_->publish(f);
    }

    // ---- 0x02F5：功率/剩余时间/循环次数/故障位 ----
    {
      can_msgs::msg::Frame f; f.id = 0x02F5; f.is_extended = false; f.is_rtr = false; f.is_error = false; f.dlc = 8;
      for (auto &b : f.data) b = 0;

      double power_W = (pack_mV_ / 1000.0) * current_A_;
      int16_t p_W = (int16_t)std::clamp((int)std::lround(power_W), -32768, 32767);

      // 计算剩余时间，考虑温度、老化和循环次数影响
      double I = std::max(0.2, std::abs(current_A_));
      
      // 温度影响因子 (20-60°C范围内，最佳温度25°C)
      double avg_temp = 0.0;
      for (const auto& temp : temps_c_) avg_temp += temp;
      avg_temp /= temps_c_.size();
      double temp_factor = 1.0;
      if (avg_temp < 25.0) {
        temp_factor = 0.8 + 0.2 * (avg_temp - 20.0) / 5.0; // 低温影响
      } else if (avg_temp > 35.0) {
        temp_factor = 1.0 - 0.3 * (avg_temp - 35.0) / 25.0; // 高温影响
      }
      
      // 老化影响因子 (基于循环次数，每1000次循环容量衰减5%)
      double cycle_count = 163.0; // 当前循环次数
      double aging_factor = 1.0 - (cycle_count / 1000.0) * 0.05;
      aging_factor = std::clamp(aging_factor, 0.7, 1.0); // 限制在70%-100%
      
      // 有效容量 = 标称容量 × 温度因子 × 老化因子
      double effective_capacity_ah = 10.0 * temp_factor * aging_factor;
      
      // 剩余时间计算 (分钟)
      double remain_min = (soc_ / 100.0) * effective_capacity_ah / I * 60.0;
      uint16_t remain_min_uint = (uint16_t)std::clamp((int)std::lround(remain_min), 0, 65535);

      pack_s16_be(&f.data[0], p_W);
      pack_u16_be(&f.data[2], remain_min_uint);
      pack_u16_be(&f.data[4], (uint16_t)cycle_count); // cycle
      pack_u16_be(&f.data[6], 0);   // fault
      pub_->publish(f);
    }

    // ---- 0x0300..：单体电压（每帧 4 节，单位 mV/BE）----
    for (int k = 0; k < (cells_ + 3) / 4; ++k) {
      can_msgs::msg::Frame f; f.id = 0x0300 + k; f.is_extended = false; f.is_rtr = false; f.is_error = false; f.dlc = 8;
      for (auto &b : f.data) b = 0;
      for (int i = 0; i < 4; ++i) {
        int idx = k * 4 + i; if (idx >= cells_) break;
        pack_u16_be(&f.data[i * 2], (uint16_t)cells_mV_[idx]);
      }
      pub_->publish(f);
    }

    // ---- 0x0310..：温度（每帧 4 个，s16，0.1℃/LSB，BE）----
    for (int k = 0; k < (temps_ + 3) / 4; ++k) {
      can_msgs::msg::Frame f; f.id = 0x0310 + k; f.is_extended = false; f.is_rtr = false; f.is_error = false; f.dlc = 8;
      for (auto &b : f.data) b = 0;
      for (int i = 0; i < 4; ++i) {
        int idx = k * 4 + i; if (idx >= temps_) break;
        int16_t t_01C = (int16_t)std::lround(temps_c_[idx] * 10.0);
        pack_s16_be(&f.data[i * 2], t_01C);
      }
      pub_->publish(f);
    }
  }

  // members
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_; std::normal_distribution<double> n_{0.0, 1.0};
  int cells_{12}, temps_{6}, pack_mV_{0}; double current_A_{-1.5}, soc_{90.0}, rate_hz_{20.0};
  // double battery_capacity_ah_{10.0}; // TODO: 可配置的电池容量参数
  std::vector<int> cells_mV_; std::vector<double> temps_c_;
  uint8_t alive_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeCanPub>());
  rclcpp::shutdown();
  return 0;
}
