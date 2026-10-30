#include "ieir_controllers/dm_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "yaml-cpp/yaml.h"

namespace ieir_controllers
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

const std::unordered_map<std::string, std::tuple<double, double, double>> & motor_limit_table()
{
  static const std::unordered_map<std::string, std::tuple<double, double, double>> table = {
    {"DM4310",  {12.5, 50.0, 10.0}},
    {"DM4340",  {12.5, 20.0, 28.0}},
    {"DM4340P", {12.5, 20.0, 28.0}},
    {"DM8009",  {12.5, 45.0, 54.0}},
  };
  return table;
}

bool parse_double(const std::string & s, double & out)
{
  try { out = std::stod(s); return true; } catch (...) { return false; }
}

bool parse_int(const std::string & s, int & out)
{
  try { out = std::stoi(s); return true; } catch (...) { return false; }
}

double sanitize(double v) { return std::isfinite(v) ? v : 0.0; }
}  // namespace

DMHardwareInterface::DMHardwareInterface() = default;

bool DMHardwareInterface::lookup_motor_limits(
  const std::string & motor_type,
  double & pos_max,
  double & vel_max,
  double & tor_max)
{
  const auto & table = motor_limit_table();
  auto it = table.find(motor_type);
  if (it == table.end()) {
    return false;
  }
  pos_max = std::get<0>(it->second);
  vel_max = std::get<1>(it->second);
  tor_max = std::get<2>(it->second);
  return true;
}

double DMHardwareInterface::wrap_to_window(double x, double center)
{
  // Wrap x into the half-open interval (center - pi, center + pi].
  double y = std::fmod(x - center + M_PI, kTwoPi);
  if (y <= 0.0) {
    y += kTwoPi;
  }
  return y - M_PI + center;
}

double DMHardwareInterface::raw_to_urdf_pos(const JointCfg & j, double raw) const
{
  const double linear = j.axis_sign * (raw - j.zero_offset);
  return j.wrap_safe ? wrap_to_window(linear, j.range_center) : linear;
}

double DMHardwareInterface::urdf_to_raw_pos(const JointCfg & j, double urdf_pos) const
{
  // Inverse of raw_to_urdf_pos (linear branch). We do not re-wrap on the raw
  // side because the motor itself accumulates the multi-turn position; the
  // commanded raw position must agree with the motor's current revolution.
  // axis_sign is +/-1 so its own inverse.
  return j.axis_sign * urdf_pos + j.zero_offset;
}

double DMHardwareInterface::urdf_to_raw_pos_near(
  const JointCfg & j, double urdf_pos, double current_raw) const
{
  const double linear = j.axis_sign * urdf_pos + j.zero_offset;
  if (!j.wrap_safe) return linear;
  // Choose the integer k such that linear + k*2pi is closest to current_raw.
  // Without this, when state_pos_ has been wrapped from a multi-turn raw
  // value the round-trip through urdf_to_raw_pos() always lands in the
  // canonical revolution (k=0), so commanding the motor to "hold its
  // current position" would force it to spin a full 2 pi to reach a
  // target on the wrong side of the wrap.
  const double diff = current_raw - linear;
  const int k = static_cast<int>(std::round(diff / kTwoPi));
  return linear + k * kTwoPi;
}

bool DMHardwareInterface::load_offsets_yaml(const std::string & path)
{
  if (path.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                "offsets_yaml is required; calibrate the W3 arms before starting control");
    return false;
  }

  std::ifstream f(path);
  if (!f.good()) {
    RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                 "offsets_yaml '%s' not readable", path.c_str());
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                 "Failed to parse offsets_yaml '%s': %s", path.c_str(), e.what());
    return false;
  }

  if (!root["offsets"] || !root["offsets"].IsSequence()) {
    RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                 "offsets_yaml '%s' has no 'offsets' sequence", path.c_str());
    return false;
  }

  std::unordered_map<std::string, std::pair<double, double>> by_name;  // name -> (zero_offset, axis_sign)
  for (const auto & e : root["offsets"]) {
    if (!e["name"]) continue;
    std::string name = e["name"].as<std::string>();
    double zo = e["zero_offset"] ? e["zero_offset"].as<double>() : 0.0;
    double sign = e["axis_sign"] ? e["axis_sign"].as<double>() : 1.0;
    if (!e["zero_offset"] || !std::isfinite(zo) ||
        (sign != 1.0 && sign != -1.0) || by_name.count(name)) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Invalid/duplicate calibration entry for '%s'", name.c_str());
      return false;
    }
    by_name[name] = {zo, sign};
  }

  size_t hit = 0;
  for (auto & j : joints_) {
    auto it = by_name.find(j.name);
    if (it != by_name.end()) {
      j.zero_offset = it->second.first;
      j.axis_sign = it->second.second;
      ++hit;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                  "Joint '%s' not found in offsets_yaml; refusing uncalibrated control",
                  j.name.c_str());
      return false;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("DMHardwareInterface"),
              "Loaded calibration for %zu/%zu joints from %s",
              hit, joints_.size(), path.c_str());
  return true;
}

hardware_interface::CallbackReturn DMHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- hardware-level params ----
  if (auto it = info_.hardware_parameters.find("offsets_yaml");
      it != info_.hardware_parameters.end()) {
    offsets_yaml_path_ = it->second;
  }
  if (auto it = info_.hardware_parameters.find("motor_topic_ns");
      it != info_.hardware_parameters.end()) {
    motor_topic_ns_ = it->second;
  }
  if (auto it = info_.hardware_parameters.find("auto_enable");
      it != info_.hardware_parameters.end()) {
    auto_enable_ = (it->second == "true" || it->second == "1");
  }

  // ---- per-joint params ----
  if (auto it = info_.hardware_parameters.find("feedback_max_age_s");
      it != info_.hardware_parameters.end()) {
    if (!parse_double(it->second, feedback_max_age_s_) ||
        !std::isfinite(feedback_max_age_s_) || feedback_max_age_s_ <= 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"), "Invalid feedback_max_age_s");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());
  joint_name_to_index_.clear();
  joints_by_channel_.clear();

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & jinfo = info_.joints[i];
    JointCfg j;
    j.name = jinfo.name;

    auto get = [&jinfo](const std::string & k) -> std::string {
      auto it = jinfo.parameters.find(k);
      return it != jinfo.parameters.end() ? it->second : std::string{};
    };

    int ch = 0, slot = 0;
    if (!parse_int(get("channel"), ch)) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Joint '%s' missing/invalid <param name=\"channel\">", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!parse_int(get("slot"), slot)) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Joint '%s' missing/invalid <param name=\"slot\">", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (slot < 0 || slot > 7) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Joint '%s' slot=%d out of range [0,7]", j.name.c_str(), slot);
      return hardware_interface::CallbackReturn::ERROR;
    }
    j.channel = ch;
    if (ch < 0 || ch > 1) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Arm channel must be 0 (can0) or 1 (can1), got %d", ch);
      return hardware_interface::CallbackReturn::ERROR;
    }
    j.slot = slot;
    j.motor_type = get("motor_type");

    double v;
    if (parse_double(get("urdf_lower"), v)) j.urdf_lower = v;
    if (parse_double(get("urdf_upper"), v)) j.urdf_upper = v;

    double tbl_pos = std::numeric_limits<double>::quiet_NaN();
    double tbl_vel = std::numeric_limits<double>::quiet_NaN();
    double tbl_tor = std::numeric_limits<double>::quiet_NaN();
    if (!lookup_motor_limits(j.motor_type, tbl_pos, tbl_vel, tbl_tor)) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Joint '%s' has unknown motor_type='%s' (expected DM4310/DM4340/DM4340P/DM8009)",
                   j.name.c_str(), j.motor_type.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    j.pos_max = tbl_pos;
    j.vel_max = tbl_vel;
    j.tor_max = tbl_tor;
    if (parse_double(get("pos_max"), v)) j.pos_max = v;
    if (parse_double(get("vel_max"), v)) j.vel_max = v;
    if (parse_double(get("tor_max"), v)) j.tor_max = v;

    if (j.urdf_upper <= j.urdf_lower) {
      RCLCPP_ERROR(rclcpp::get_logger("DMHardwareInterface"),
                   "Joint '%s' urdf_upper (%.3f) <= urdf_lower (%.3f)",
                   j.name.c_str(), j.urdf_upper, j.urdf_lower);
      return hardware_interface::CallbackReturn::ERROR;
    }
    j.range_center = 0.5 * (j.urdf_lower + j.urdf_upper);
    j.wrap_safe = (j.urdf_upper - j.urdf_lower) <= kTwoPi - 1e-6;

    joint_name_to_index_[j.name] = i;
    joints_by_channel_[j.channel].push_back(i);
    joints_.push_back(std::move(j));
  }

  unique_channels_.clear();
  for (const auto & kv : joints_by_channel_) {
    unique_channels_.push_back(kv.first);
  }

  // ---- storage allocation ----
  const size_t n = joints_.size();
  state_pos_.assign(n, 0.0);
  state_vel_.assign(n, 0.0);
  state_eff_.assign(n, 0.0);
  feedback_source_ns_.assign(n, 0);
  feedback_receive_steady_ns_.assign(n, 0);
  feedback_packet_valid_.assign(n, false);
  feedback_source_time_.assign(n, std::numeric_limits<double>::quiet_NaN());
  feedback_source_age_.assign(n, std::numeric_limits<double>::infinity());
  feedback_receive_age_.assign(n, std::numeric_limits<double>::infinity());
  feedback_valid_.assign(n, 0.0);
  cmd_pos_.assign(n, 0.0);
  cmd_vel_.assign(n, 0.0);
  cmd_eff_.assign(n, 0.0);
  cmd_kp_.assign(n, 0.0);
  cmd_kd_.assign(n, 0.0);
  last_err_.assign(n, -1);
  last_pos_raw_.assign(n, 0.0);

  // ---- load calibration ----
  if (!load_offsets_yaml(offsets_yaml_path_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- log summary ----
  for (const auto & j : joints_) {
    RCLCPP_INFO(rclcpp::get_logger("DMHardwareInterface"),
                "  %-12s ch%d.slot%d  type=%-8s offset=%+.4f sign=%+.0f urdf=[%+.3f,%+.3f] center=%+.3f wrap_safe=%s pos_max=%.1f vel_max=%.1f tor_max=%.1f",
                j.name.c_str(), j.channel, j.slot, j.motor_type.c_str(),
                j.zero_offset, j.axis_sign, j.urdf_lower, j.urdf_upper,
                j.range_center, j.wrap_safe ? "true" : "false",
                j.pos_max, j.vel_max, j.tor_max);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DMHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // All callbacks stay on the hardware owner thread. No new shared state races.
  feedback_executor_.reset();
  feedback_rt_pub_.reset();
  node_ = std::make_shared<rclcpp::Node>("dm_hardware_interface");

  const std::string state_t = motor_topic_ns_ + "/state";
  const std::string cmd_t = motor_topic_ns_ + "/commands";
  const auto feedback_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  state_sub_ = node_->create_subscription<w3_robot_bridge::msg::MotorStateArray>(
    state_t, feedback_qos,
    [this](w3_robot_bridge::msg::MotorStateArray::SharedPtr msg) {
      on_motor_state(msg);
    });
  cmd_pub_ = node_->create_publisher<w3_robot_bridge::msg::MotorCommandArray>(
    cmd_t, rclcpp::QoS(50).best_effort());
  feedback_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "~/feedback_state", feedback_qos);
  feedback_rt_pub_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
    feedback_pub_);
  auto & output = feedback_rt_pub_->msg_;
  for (const auto & joint : joints_) output.name.push_back(joint.name);
  output.position.resize(joints_.size());
  output.velocity.resize(joints_.size());
  output.effort.resize(joints_.size());
  feedback_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  feedback_executor_->add_node(node_);
  latest_frame_stamp_ns_ = published_frame_stamp_ns_ = 0;
  std::fill(feedback_source_ns_.begin(), feedback_source_ns_.end(), 0);
  std::fill(feedback_receive_steady_ns_.begin(), feedback_receive_steady_ns_.end(), 0);
  std::fill(feedback_packet_valid_.begin(), feedback_packet_valid_.end(), false);
  refresh_feedback_metadata();

  for (int ch : unique_channels_) {
    state_seen_[ch] = false;
  }
  RCLCPP_INFO(node_->get_logger(), "W3 bridge: sub %s   pub %s",
              state_t.c_str(), cmd_t.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DMHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(joints_.size() * 7);
  for (size_t i = 0; i < joints_.size(); ++i) {
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &state_pos_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &state_vel_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_EFFORT,   &state_eff_[i]);
    out.emplace_back(joints_[i].name, "feedback_source_time", &feedback_source_time_[i]);
    out.emplace_back(joints_[i].name, "feedback_source_age", &feedback_source_age_[i]);
    out.emplace_back(joints_[i].name, "feedback_receive_age", &feedback_receive_age_[i]);
    out.emplace_back(joints_[i].name, "feedback_valid", &feedback_valid_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
DMHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(joints_.size() * 5);
  for (size_t i = 0; i < joints_.size(); ++i) {
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_EFFORT,   &cmd_eff_[i]);
    out.emplace_back(joints_[i].name, "stiffness", &cmd_kp_[i]);
    out.emplace_back(joints_[i].name, "damping",   &cmd_kd_[i]);
  }
  return out;
}

void DMHardwareInterface::publish_enable_all(bool enable)
{
  w3_robot_bridge::msg::MotorCommandArray msg;
  msg.header.stamp = node_->now();
  msg.commands.reserve(joints_.size());
  for (const auto & j : joints_) {
    w3_robot_bridge::msg::MotorCommand m;
    m.channel = static_cast<uint8_t>(j.channel);
    m.motor_index = static_cast<uint8_t>(j.slot);
    m.mode = enable ?
      w3_robot_bridge::msg::MotorCommand::MODE_ENABLE :
      w3_robot_bridge::msg::MotorCommand::MODE_DISABLE;
    msg.commands.push_back(m);
  }
  cmd_pub_->publish(msg);
}

void DMHardwareInterface::publish_zero_command_all()
{
  w3_robot_bridge::msg::MotorCommandArray msg;
  msg.header.stamp = node_->now();
  msg.commands.reserve(joints_.size());
  for (const auto & j : joints_) {
    w3_robot_bridge::msg::MotorCommand m;
    m.channel = static_cast<uint8_t>(j.channel);
    m.motor_index = static_cast<uint8_t>(j.slot);
    m.mode = w3_robot_bridge::msg::MotorCommand::MODE_RUN;
    m.position = 0.0f;
    m.velocity = 0.0f;
    m.kp = 0.0f;
    m.kd = 0.0f;
    m.torque = 0.0f;
    msg.commands.push_back(m);
  }
  cmd_pub_->publish(msg);
}

hardware_interface::CallbackReturn DMHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands to safe defaults so an effort-only controller does not
  // accidentally generate spring/damping torques via stale kp/kd.
  std::fill(cmd_pos_.begin(), cmd_pos_.end(), 0.0);
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0.0);
  std::fill(cmd_eff_.begin(), cmd_eff_.end(), 0.0);
  std::fill(cmd_kp_.begin(), cmd_kp_.end(), 0.0);
  std::fill(cmd_kd_.begin(), cmd_kd_.end(), 0.0);

  quiescing_ = false;

  if (!auto_enable_) {
    RCLCPP_INFO(node_->get_logger(),
                "auto_enable=false; user must publish enable manually");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Refresh enable plus zero impedance at 50 Hz until motor feedback confirms.
  // W3 schedules CAN transmission separately; write() is not running yet.
  static constexpr int DM_ERR_ENABLED = 1;
  const double timeout_s = 5.0;
  const auto loop_period = std::chrono::milliseconds(20);  // 50 Hz both
  const auto t0 = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(),
              "ENABLE: 50Hz zero-cmd + 50Hz motor_enable publish until all "
              "%zu joint(s) report err=%d (timeout %.1fs)...",
              joints_.size(), DM_ERR_ENABLED, timeout_s);
  while (rclcpp::ok()) {
    poll_feedback();
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now - t0).count();
    if (elapsed_s > timeout_s) break;
    // Both at 50 Hz: ENABLE keeps hammering the DM_ENABLE special-byte
    // sequence at every motor until each one transitions to MIT, then
    // zero-cmd immediately overwrites W3's transient enable damping command.
    publish_enable_all(true);
    publish_zero_command_all();
    bool all_enabled = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (last_err_[i] != DM_ERR_ENABLED || feedback_valid_[i] != 1.0) {
        all_enabled = false;
        break;
      }
    }
    if (all_enabled) {
      RCLCPP_INFO(node_->get_logger(),
                  "ENABLED (all %zu motor(s) report err=%d)",
                  joints_.size(), DM_ERR_ENABLED);
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(loop_period);
  }
  // Timeout: list per-joint err for diagnosis and fail activation so
  // ros2_control surfaces the error to the user.
  std::string offenders;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (last_err_[i] != DM_ERR_ENABLED || feedback_valid_[i] != 1.0) {
      if (!offenders.empty()) offenders += ", ";
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s(err=%d,fresh=%d)",
                    joints_[i].name.c_str(), last_err_[i], feedback_valid_[i] == 1.0);
      offenders += buf;
    }
  }
  RCLCPP_ERROR(node_->get_logger(),
               "ENABLE timed out after %.1fs; not enabled: %s",
               timeout_s, offenders.c_str());
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn DMHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Force write() into zero-cmd mode for the rest of this transition --
  // ros2_control's update loop may still race with this callback.
  quiescing_ = true;

  // Burst zero-cmd so motors are passive when DISABLE lands.
  for (int i = 0; i < 3; ++i) {
    publish_zero_command_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!auto_enable_) {
    RCLCPP_INFO(node_->get_logger(),
                "Deactivated; commands zeroed (auto_enable=false, no DISABLE sent)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ---- synchronous DISABLE: 5 Hz publish until every joint reports
  // err == DM_ERR_DISABLED (0). We *keep* sending cmd via write() (still
  // being called by CM until on_deactivate returns) -- DM is
  // request-response, so without cmd we'd never see a fresh err code.
  // quiescing_ above forces those cmds to zero so they can't inject torque.
  //
  // Note: we explicitly poll for err==0 (disabled) rather than err!=1
  // (anything-but-enabled). Error codes 8..14 (over-volt / over-current /
  // over-temp / etc.) also satisfy err!=1 but they do NOT mean the motor
  // has cleanly disabled -- treating them as "DISABLED" would silently
  // swallow a real fault on shutdown.
  static constexpr int DM_ERR_DISABLED = 0;
  const double timeout_s = 5.0;
  const double publish_period_s = 0.05;
  const auto t0 = std::chrono::steady_clock::now();
  auto last_pub = t0 - std::chrono::seconds(1);
  RCLCPP_INFO(node_->get_logger(),
              "DISABLE: %.0fHz FD pulse until all %zu joint(s) report err=%d "
              "(timeout %.1fs)...",
              1.0 / publish_period_s, joints_.size(), DM_ERR_DISABLED, timeout_s);
  while (rclcpp::ok()) {
    poll_feedback();
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now - t0).count();
    if (elapsed_s > timeout_s) break;
    if (std::chrono::duration<double>(now - last_pub).count() >= publish_period_s) {
      publish_enable_all(false);
      last_pub = now;
    }
    bool all_off = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (last_err_[i] != DM_ERR_DISABLED || feedback_valid_[i] != 1.0) {
        all_off = false;
        break;
      }
    }
    if (all_off) {
      RCLCPP_INFO(node_->get_logger(),
                  "DISABLED (all %zu motor(s) report err=%d)",
                  joints_.size(), DM_ERR_DISABLED);
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  // Timeout: list per-joint err for diagnosis. Distinguish "still enabled"
  // (err==1) from "stuck in fault" (err in 8..14) so the user knows
  // whether to power-cycle the affected motor.
  std::string offenders;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (last_err_[i] != DM_ERR_DISABLED || feedback_valid_[i] != 1.0) {
      if (!offenders.empty()) offenders += ", ";
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s(err=%d,fresh=%d)",
                    joints_[i].name.c_str(), last_err_[i], feedback_valid_[i] == 1.0);
      offenders += buf;
    }
  }
  RCLCPP_WARN(node_->get_logger(),
              "DISABLE timed out after %.1fs; not disabled: %s",
              timeout_s, offenders.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

void DMHardwareInterface::on_motor_state(
  const w3_robot_bridge::msg::MotorStateArray::SharedPtr msg)
{
  if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U) return;
  const int64_t source_ns = int64_t(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
  const auto now_ros_ns = node_->get_clock()->now().nanoseconds();
  // Do not roll back a newer cache or invent a time for an unstamped packet.
  if (source_ns <= 0 || source_ns <= latest_frame_stamp_ns_ || source_ns > now_ros_ns ||
      (now_ros_ns - source_ns) * 1e-9 > feedback_max_age_s_) return;
  latest_frame_stamp_ns_ = source_ns;
  const auto received_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  std::fill(feedback_packet_valid_.begin(), feedback_packet_valid_.end(), false);
  for (const auto & m : msg->motors) {
    const int ch = static_cast<int>(m.channel);
    state_seen_[ch] = true;
    auto by_channel = joints_by_channel_.find(ch);
    if (by_channel == joints_by_channel_.end()) {
      continue;
    }
    for (size_t idx : by_channel->second) {
      const auto & j = joints_[idx];
      if (j.slot != static_cast<int>(m.motor_index)) {
        continue;
      }
      if (!m.online || !std::isfinite(m.position) ||
          !std::isfinite(m.velocity) || !std::isfinite(m.torque)) {
        continue;
      }
      if (m.header.stamp.sec < 0 || m.header.stamp.nanosec >= 1000000000U) continue;
      const int64_t motor_stamp = int64_t(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;
      const auto motor_source_ns = motor_stamp > 0 ? motor_stamp : source_ns;
      if (motor_source_ns <= feedback_source_ns_[idx] || motor_source_ns > now_ros_ns ||
          (now_ros_ns - motor_source_ns) * 1e-9 > feedback_max_age_s_) continue;
      last_err_[idx] = static_cast<int>(m.error_flags);
      last_pos_raw_[idx] = static_cast<double>(m.position);
      state_pos_[idx] = raw_to_urdf_pos(j, static_cast<double>(m.position));
      state_vel_[idx] = j.axis_sign * static_cast<double>(m.velocity);
      state_eff_[idx] = j.axis_sign * static_cast<double>(m.torque);
      feedback_source_ns_[idx] = motor_source_ns;
      feedback_receive_steady_ns_[idx] = received_ns;
      feedback_packet_valid_[idx] = true;
      break;
    }
  }
}

void DMHardwareInterface::refresh_feedback_metadata()
{
  const auto now_ros_ns = node_->get_clock()->now().nanoseconds();
  const auto now_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  for (size_t i = 0; i < joints_.size(); ++i) {
    const bool seen = feedback_source_ns_[i] > 0;
    feedback_source_time_[i] = seen ? feedback_source_ns_[i] * 1e-9 :
      std::numeric_limits<double>::quiet_NaN();
    feedback_source_age_[i] = seen ? (now_ros_ns - feedback_source_ns_[i]) * 1e-9 :
      std::numeric_limits<double>::infinity();
    feedback_receive_age_[i] = seen ? (now_steady_ns - feedback_receive_steady_ns_[i]) * 1e-9 :
      std::numeric_limits<double>::infinity();
    feedback_valid_[i] = feedback_packet_valid_[i] && feedback_source_age_[i] >= 0.0 &&
      feedback_source_age_[i] <= feedback_max_age_s_ &&
      feedback_receive_age_[i] <= feedback_max_age_s_ ? 1.0 : 0.0;
  }
}

void DMHardwareInterface::publish_feedback_state()
{
  if (latest_frame_stamp_ns_ <= published_frame_stamp_ns_ || joints_.empty() ||
      !std::all_of(feedback_valid_.begin(), feedback_valid_.end(), [](double x) {return x == 1.0;})) return;
  if (!feedback_rt_pub_->trylock()) return;
  auto & output = feedback_rt_pub_->msg_;
  // W3 currently stamps all joints with array publication time. For a future
  // mixed snapshot, conservatively label it with its oldest joint source.
  const auto stamp_ns = *std::min_element(feedback_source_ns_.begin(), feedback_source_ns_.end());
  output.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  output.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  std::copy(state_pos_.begin(), state_pos_.end(), output.position.begin());
  std::copy(state_vel_.begin(), state_vel_.end(), output.velocity.begin());
  std::copy(state_eff_.begin(), state_eff_.end(), output.effort.begin());
  published_frame_stamp_ns_ = latest_frame_stamp_ns_;
  feedback_rt_pub_->unlockAndPublish();
}

void DMHardwareInterface::poll_feedback()
{
  // Collect readiness once; no unbounded draining of a growing FIFO.
  feedback_executor_->spin_some(std::chrono::nanoseconds(0));
  refresh_feedback_metadata();
  publish_feedback_state();
}

hardware_interface::return_type DMHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // State interfaces are populated in on_motor_state (sentinel-filtered);
  // here we just dispatch any pending state messages so the cache is
  // up-to-date before controllers' update() runs.
  poll_feedback();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DMHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Emit one W3 MotorCommandArray carrying all active arm joints.
  //
  // While quiescing_ is true (set by on_deactivate), force every field to
  // zero so a still-active controller cannot inject torques into a motor
  // that we are simultaneously trying to disable.
  w3_robot_bridge::msg::MotorCommandArray msg;
  msg.header.stamp = node_->now();
  msg.commands.reserve(joints_.size());
  for (int ch : unique_channels_) {
    for (size_t idx : joints_by_channel_[ch]) {
      const auto & j = joints_[idx];
      double pos_urdf, vel_urdf, eff_urdf, kp, kd;
      if (quiescing_) {
        pos_urdf = vel_urdf = eff_urdf = kp = kd = 0.0;
      } else {
        pos_urdf = sanitize(cmd_pos_[idx]);
        vel_urdf = sanitize(cmd_vel_[idx]);
        eff_urdf = sanitize(cmd_eff_[idx]);
        kp       = std::max(0.0, sanitize(cmd_kp_[idx]));
        kd       = std::max(0.0, sanitize(cmd_kd_[idx]));
      }

      // Safety mirror for pure-torque mode (kp == kd == 0):
      //   Mirror measured raw position when only torque is controlled.
      //   This keeps the position field continuous across mode switches.
      double pos_raw, vel_raw;
      if (!quiescing_ && kp == 0.0 && kd == 0.0) {
        pos_raw = last_pos_raw_[idx];
        vel_raw = 0.0;
      } else {
        // Use the multi-turn-aware inverse so the commanded raw position
        // tracks the motor's current revolution. urdf_to_raw_pos() is the
        // canonical (k=0) inverse, which is incorrect when state_pos_ was
        // wrapped from a multi-turn raw value: the controller's hold_pos_
        // would round-trip into the wrong revolution and command a 2 pi
        // slam toward a hard stop.
        pos_raw = urdf_to_raw_pos_near(j, pos_urdf, last_pos_raw_[idx]);
        vel_raw = j.axis_sign * vel_urdf;
      }
      const double eff_clamped = std::clamp(eff_urdf, -j.tor_max, j.tor_max);
      const double tau_raw = j.axis_sign * eff_clamped;

      w3_robot_bridge::msg::MotorCommand m;
      m.channel = static_cast<uint8_t>(j.channel);
      m.motor_index = static_cast<uint8_t>(j.slot);
      m.mode = w3_robot_bridge::msg::MotorCommand::MODE_RUN;
      m.position = static_cast<float>(pos_raw);
      m.velocity = static_cast<float>(vel_raw);
      m.kp = static_cast<float>(kp);
      m.kd = static_cast<float>(kd);
      m.torque = static_cast<float>(tau_raw);
      msg.commands.push_back(m);
    }
  }
  cmd_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

}  // namespace ieir_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ieir_controllers::DMHardwareInterface,
  hardware_interface::SystemInterface)
