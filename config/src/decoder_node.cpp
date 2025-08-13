
#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>

using std::placeholders::_1;
using can_msgs::msg::Frame;
using sensor_msgs::msg::BatteryState;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;
using std_msgs::msg::UInt16;
using std_msgs::msg::UInt32;

static inline uint16_t U16LE(uint8_t b0, uint8_t b1){ return (uint16_t)b0 | ((uint16_t)b1<<8); }
static inline int16_t  S16LE(uint8_t b0, uint8_t b1){
  uint16_t v = U16LE(b0,b1);
  return (v & 0x8000) ? (int16_t)(v - 0x10000) : (int16_t)v;
}

class BmsDecoder : public rclcpp::Node {
public:
  BmsDecoder() : rclcpp::Node("bms_can_decoder") {
    this->declare_parameter<std::string>("from_topic", "/from_can");
    this->declare_parameter<int>("max_cells", 128);
    this->declare_parameter<double>("publish_full_arrays_hz", 2.0);

    auto topic = this->get_parameter("from_topic").as_string();
    sub_ = this->create_subscription<Frame>(topic, 500, std::bind(&BmsDecoder::onFrame, this, _1));

    pub_batt_     = this->create_publisher<BatteryState>("/bms/batt_st1", 10);
    pub_alarms_   = this->create_publisher<DiagnosticArray>("/bms/alarms", 10);
    pub_faults_   = this->create_publisher<DiagnosticArray>("/bms/faults", 10);
    pub_limits_   = this->create_publisher<DiagnosticArray>("/bms/limits", 10);
    pub_info_     = this->create_publisher<DiagnosticArray>("/bms/info", 10);
    pub_switches_ = this->create_publisher<UInt32>("/bms/switches", 10);
    pub_charge_   = this->create_publisher<DiagnosticArray>("/bms/charge_request", 10);
    pub_ctrl_     = this->create_publisher<DiagnosticArray>("/bms/ctrl_info", 10);

    pub_vsum_ = this->create_publisher<Float32MultiArray>("/bms/cell/volt_summary", 10);
    pub_tsum_ = this->create_publisher<Float32MultiArray>("/bms/cell/temp_summary", 10);
    pub_idx_min_ = this->create_publisher<UInt16>("/bms/cell/index_min", 10);
    pub_idx_max_ = this->create_publisher<UInt16>("/bms/cell/index_max", 10);
    pub_vfull_ = this->create_publisher<Float32MultiArray>("/bms/cell/voltages", 10);
    pub_tfull_ = this->create_publisher<Float32MultiArray>("/bms/cell/temps", 10);
    pub_rated_ah_ = this->create_publisher<Float32>("/bms/rated_capacity_ah", 10);
    pub_nom_v_    = this->create_publisher<Float32>("/bms/nominal_voltage_v", 10);

    int max_cells = this->get_parameter("max_cells").as_int();
    cell_volt_.assign(max_cells, 0.0f);
    cell_temp_.assign(max_cells, 0.0f);

    double hz = this->get_parameter("publish_full_arrays_hz").as_double();
    double period = (hz > 0.1) ? (1.0 / hz) : 10.0;
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period),
             std::bind(&BmsDecoder::publishFullArrays, this));

    RCLCPP_INFO(this->get_logger(), "Listening CAN frames on %s (expect 250 kbps)", topic.c_str());
  }

private:
  // Helpers
  DiagnosticArray makeDiag(const std::string& name, int level, const std::vector<std::pair<std::string,std::string>>& kvs, const std::string& msg=""){
    DiagnosticArray d;
    DiagnosticStatus st;
    st.level = level;
    st.name = name;
    st.message = msg;
    for(const auto& kv : kvs){
      KeyValue kvp; kvp.key = kv.first; kvp.value = kv.second; st.values.push_back(kvp);
    }
    d.status.push_back(st);
    return d;
  }

  void publishFullArrays(){
    Float32MultiArray va; for(float v : cell_volt_) if(v != 0.0f) va.data.push_back(v); pub_vfull_->publish(va);
    Float32MultiArray ta; for(float t : cell_temp_) if(t != 0.0f) ta.data.push_back(t); pub_tfull_->publish(ta);
  }

  void onFrame(const Frame& msg){
    const auto& d = msg.data;
    if(d.size() < 8) return;

    if(!msg.is_extended){
      // 0x02F4 BATT_ST1
      if(msg.id == 0x02F4){
        float pack_v = (float)U16LE(d[0],d[1]) * 0.01f;
        float pack_i = (float)S16LE(d[2],d[3]) * 0.1f;
        float soc_pct = (float)d[4] * 0.5f;
        float soh_pct = (float)d[5] * 0.5f;
        BatteryState out;
        out.voltage = pack_v;
        out.current = pack_i;
        out.percentage = soc_pct / 100.0f;
        out.power_supply_status = (pack_i < 0) ? BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
                                               : BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        out.power_supply_health = (soh_pct >= 80.0f) ? BatteryState::POWER_SUPPLY_HEALTH_GOOD
                                                     : BatteryState::POWER_SUPPLY_HEALTH_DEAD;
        pub_batt_->publish(out);
        return;
      }
      // 0x04F4 CELL_VOLT summary
      if(msg.id == 0x04F4){
        float vmin = (float)U16LE(d[0],d[1]) * 0.001f;
        float vmax = (float)U16LE(d[2],d[3]) * 0.001f;
        uint16_t idx_min = d[4];
        uint16_t idx_max = d[5];
        float vdelta = (float)U16LE(d[6],d[7]) * 0.001f;
        Float32MultiArray arr; arr.data = {vmin, vmax, vdelta}; pub_vsum_->publish(arr);
        UInt16 m1; m1.data = idx_min; pub_idx_min_->publish(m1);
        UInt16 m2; m2.data = idx_max; pub_idx_max_->publish(m2);
        return;
      }
      // 0x05F4 CELL_TEMP summary
      if(msg.id == 0x05F4){
        float tmin = (float)S16LE(d[0],d[1]) * 0.1f;
        float tmax = (float)S16LE(d[2],d[3]) * 0.1f;
        float tavg = (float)S16LE(d[4],d[5]) * 0.1f;
        Float32MultiArray arr; arr.data = {tmin, tmax, tavg}; pub_tsum_->publish(arr);
        return;
      }
      // 0x07F4 ALM_INFO
      if(msg.id == 0x07F4){
        int level = d[0];
        int domain = d[1];
        uint16_t code = U16LE(d[2],d[3]);
        int newf = d[4], recov = d[5];
        static int map[4] = {DiagnosticStatus::OK, DiagnosticStatus::WARN, DiagnosticStatus::ERROR, DiagnosticStatus::STALE};
        int lvl = (level>=0 && level<4)? map[level] : DiagnosticStatus::WARN;
        pub_alarms_->publish(makeDiag("BMS Alarm", lvl, {
          {"domain", std::to_string(domain)},
          {"code_hex", (std::stringstream() << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << code).str()},
          {"new", std::to_string(newf)},
          {"recovered", std::to_string(recov)}
        }));
        return;
      }
      return;
    }

    // -------- Extended frames --------
    // 0x18F128F4 BATT_ST2
    if(msg.id == 0x18F128F4){
      float max_chg = (float)U16LE(d[0],d[1]) * 0.1f;
      float max_dsg = (float)U16LE(d[2],d[3]) * 0.1f;
      uint16_t remain_s = U16LE(d[4],d[5]);
      uint8_t work_mode = d[6];
      pub_limits_->publish(makeDiag("BMS Limits", DiagnosticStatus::OK, {
        {"MaxChgCurr_A", (std::stringstream() << std::fixed << std::setprecision(1) << max_chg).str()},
        {"MaxDsgCurr_A", (std::stringstream() << std::fixed << std::setprecision(1) << max_dsg).str()},
        {"RemainTime_s", std::to_string(remain_s)},
        {"WorkMode", (std::stringstream() << "0x" << std::hex << std::uppercase << (int)work_mode).str()}
      }));
      return;
    }
    // 0x18F228F4 ALL_TEMP (paged)
    if(msg.id == 0x18F228F4){
      uint8_t page = d[0], count = d[1];
      for(uint8_t i=0;i<count;i++){
        size_t off = 2 + i*2;
        if(off+1 < d.size()){
          float t = (float)S16LE(d[off], d[off+1]) * 0.1f;
          size_t idx = page*count + i;
          if(idx < cell_temp_.size()) cell_temp_[idx] = t;
        }
      }
      return;
    }
    // 0x18F328F4 BMSERR_INFO
    if(msg.id == 0x18F328F4){
      uint8_t page = d[0], count = d[1];
      std::vector<std::pair<std::string,std::string>> kv = {{"page", std::to_string(page)}, {"count", std::to_string(count)}};
      for(uint8_t i=0;i<count;i++){
        size_t off = 2 + i*2;
        if(off+1 < d.size()){
          uint16_t code = U16LE(d[off], d[off+1]);
          std::stringstream ss; ss << "0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << code;
          kv.emplace_back("err_"+std::to_string(i), ss.str());
        }
      }
      pub_faults_->publish(makeDiag("BMS Faults", DiagnosticStatus::ERROR, kv));
      return;
    }
    // 0x18F428F4 BMS_INFO
    if(msg.id == 0x18F428F4){
      float rated_ah = (float)U16LE(d[0],d[1]) * 0.1f;
      float nominal_v = (float)U16LE(d[2],d[3]) * 0.01f;
      uint8_t hw = d[4], sw = d[5];
      Float32 fa; fa.data = rated_ah; pub_rated_ah_->publish(fa);
      Float32 fv; fv.data = nominal_v; pub_nom_v_->publish(fv);
      pub_info_->publish(makeDiag("BMS Info", DiagnosticStatus::OK, {
        {"HW_Ver", std::to_string(hw)},
        {"SW_Ver", std::to_string(sw)},
        {"Rated_Ah", (std::stringstream() << rated_ah).str()},
        {"Nominal_V", (std::stringstream() << nominal_v).str()}
      }));
      return;
    }
    // 0x18F528F4 BMSSwSta
    if(msg.id == 0x18F528F4){
      uint32_t mask = (uint32_t)d[0] | ((uint32_t)d[1]<<8) | ((uint32_t)d[2]<<16) | ((uint32_t)d[3]<<24);
      UInt32 m; m.data = mask; pub_switches_->publish(m);
      std::stringstream ss; ss << "0x" << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << mask;
      pub_info_->publish(makeDiag("BMS Switches", DiagnosticStatus::OK, {{"bitmap_hex", ss.str()}}));
      return;
    }
    // 0x18E028F4 CELLVOL (paged cell voltages)
    if(msg.id == 0x18E028F4){
      uint8_t page = d[0], count = d[1];
      for(uint8_t i=0;i<count;i++){
        size_t off = 2 + i*2;
        if(off+1 < d.size()){
          float v = (float)U16LE(d[off], d[off+1]) * 0.001f;
          size_t idx = page*count + i;
          if(idx < cell_volt_.size()) cell_volt_[idx] = v;
        }
      }
      return;
    }
    // 0x1806E5F4 BMSChg_INFO
    if(msg.id == 0x1806E5F4){
      float max_v = (float)U16LE(d[0],d[1]) * 0.01f;
      float max_i = (float)U16LE(d[2],d[3]) * 0.1f;
      uint8_t en = d[4] & 0x01;
      uint8_t mode = d[5];
      pub_charge_->publish(makeDiag("BMS ChargeReq", DiagnosticStatus::OK, {
        {"MaxChgVolt_V", (std::stringstream() << max_v).str()},
        {"MaxChgCurr_A", (std::stringstream() << max_i).str()},
        {"Enable", std::to_string(en)},
        {"Mode", std::to_string(mode)}
      }));
      return;
    }
    // 0x18F0F428 Ctrl_INFO (log raw fields)
    if(msg.id == 0x18F0F428){
      std::vector<std::pair<std::string,std::string>> kv;
      for(size_t i=0;i<8;i++){
        kv.emplace_back("raw"+std::to_string(i), std::to_string(d[i]));
      }
      pub_ctrl_->publish(makeDiag("BMS Ctrl", DiagnosticStatus::OK, kv));
      return;
    }
  }

  // Members
  rclcpp::Subscription<Frame>::SharedPtr sub_;
  rclcpp::Publisher<BatteryState>::SharedPtr pub_batt_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_alarms_, pub_faults_, pub_limits_, pub_info_, pub_charge_, pub_ctrl_;
  rclcpp::Publisher<UInt32>::SharedPtr pub_switches_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_vsum_, pub_tsum_, pub_vfull_, pub_tfull_;
  rclcpp::Publisher<UInt16>::SharedPtr pub_idx_min_, pub_idx_max_;
  rclcpp::Publisher<Float32>::SharedPtr pub_rated_ah_, pub_nom_v_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<float> cell_volt_, cell_temp_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BmsDecoder>());
  rclcpp::shutdown();
  return 0;
}
