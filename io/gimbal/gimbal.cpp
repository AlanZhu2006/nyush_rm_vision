#include "gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
int read_sign_or_default(const YAML::Node & yaml, const char * key, int default_value)
{
  if (!yaml[key]) return default_value;

  auto sign = yaml[key].as<int>();
  if (sign != 1 && sign != -1) {
    tools::logger()->error("[Gimbal] {} must be either 1 or -1, got {}", key, sign);
    exit(1);
  }
  return sign;
}

Eigen::Vector3d quaternion_to_protocol_ypr(const Eigen::Quaterniond & q)
{
  const double w = q.w();
  const double x = q.x();
  const double y = q.y();
  const double z = q.z();

  const double yaw = std::atan2(2.0 * (w * z + x * y), 2.0 * (w * w + x * x) - 1.0);
  const double pitch = std::atan2(2.0 * (w * x + y * z), 2.0 * (w * w + z * z) - 1.0);
  const double roll = std::asin(std::clamp(2.0 * (w * y - x * z), -1.0, 1.0));

  return {yaw, pitch, roll};
}

Eigen::Quaterniond protocol_ypr_to_quaternion(double yaw, double pitch, double roll)
{
  // Lower-machine firmware uses yaw(z), pitch(x), roll(y).
  // tools::rotation_matrix uses yaw(z), pitch(y), roll(x), so swap pitch/roll here.
  return Eigen::Quaterniond(tools::rotation_matrix({yaw, roll, pitch})).normalized();
}
}  // namespace

Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  if (yaml["gimbal_protocol_adapter"]) {
    const auto & adapter_yaml = yaml["gimbal_protocol_adapter"];
    if (adapter_yaml["enabled"]) adapter_.enabled = adapter_yaml["enabled"].as<bool>();
    adapter_.rx_yaw_sign = read_sign_or_default(adapter_yaml, "rx_yaw_sign", 1);
    adapter_.rx_pitch_sign = read_sign_or_default(adapter_yaml, "rx_pitch_sign", 1);
    adapter_.rx_roll_sign = read_sign_or_default(adapter_yaml, "rx_roll_sign", 1);
    adapter_.tx_yaw_sign = read_sign_or_default(adapter_yaml, "tx_yaw_sign", 1);
    adapter_.tx_pitch_sign = read_sign_or_default(adapter_yaml, "tx_pitch_sign", 1);
  }

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(115200);
    serial_.setTimeout(serial::Timeout::max(), 1000, 0, 1000, 0);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  constexpr auto kInitialPacketTimeout = std::chrono::milliseconds(1500);
  const auto wait_begin = std::chrono::steady_clock::now();
  while (!quit_ && !first_packet_received_.load() &&
         (std::chrono::steady_clock::now() - wait_begin) < kInitialPacketTimeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!first_packet_received_.load()) {
    tools::logger()->warn(
      "[Gimbal] Timed out waiting for first q after {} ms; continuing with identity IMU until "
      "packets arrive.",
      kInitialPacketTimeout.count());
  }
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  if (!first_packet_received_.load() || queue_.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_q_;
  }

  while (true) {
    auto [q_a, t_a] = queue_.pop();
    if (queue_.empty()) return q_a;

    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    if (std::abs(t_ab) < 1e-6) return q_b;

    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

VisionToGimbal Gimbal::adapt_tx(const VisionToGimbal & packet) const
{
  if (!adapter_.enabled) return packet;

  auto adapted = packet;
  adapted.yaw = adapter_.tx_yaw_sign * packet.yaw;
  adapted.yaw_vel = adapter_.tx_yaw_sign * packet.yaw_vel;
  adapted.yaw_acc = adapter_.tx_yaw_sign * packet.yaw_acc;
  adapted.pitch = adapter_.tx_pitch_sign * packet.pitch;
  adapted.pitch_vel = adapter_.tx_pitch_sign * packet.pitch_vel;
  adapted.pitch_acc = adapter_.tx_pitch_sign * packet.pitch_acc;
  return adapted;
}

Eigen::Quaterniond Gimbal::adapt_rx_quaternion(const Eigen::Quaterniond & q) const
{
  if (!adapter_.enabled) return q;

  auto protocol_ypr = quaternion_to_protocol_ypr(q);
  return protocol_ypr_to_quaternion(
    adapter_.rx_yaw_sign * protocol_ypr[0], adapter_.rx_pitch_sign * protocol_ypr[1],
    adapter_.rx_roll_sign * protocol_ypr[2]);
}

void Gimbal::send(io::VisionToGimbal vision_to_gimbal)
{
  auto adapted = adapt_tx(vision_to_gimbal);

  tx_data_.mode = adapted.mode;
  tx_data_.yaw = adapted.yaw;
  tx_data_.yaw_vel = adapted.yaw_vel;
  tx_data_.yaw_acc = adapted.yaw_acc;
  tx_data_.pitch = adapted.pitch;
  tx_data_.pitch_vel = adapted.pitch_vel;
  tx_data_.pitch_acc = adapted.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  send(io::VisionToGimbal{
    {'S', 'P'}, static_cast<uint8_t>(control ? (fire ? 2 : 1) : 0), yaw, yaw_vel, yaw_acc, pitch,
    pitch_vel, pitch_acc, 0});
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;
  int packet_count = 0;
  auto last_packet_time = std::chrono::steady_clock::now();

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    packet_count++;
    if (packet_count % 100 == 0) {
      auto now = std::chrono::steady_clock::now();
      auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_packet_time).count();
      auto freq = 100.0 / (dt / 1000.0);
      tools::logger()->info("[Gimbal] Received {} packets, freq={:.1f}Hz", packet_count, freq);
      last_packet_time = now;
    }

    Eigen::Quaterniond q_proto(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    auto q = adapt_rx_quaternion(q_proto);
    queue_.push({q, t});

    if (!first_packet_received_.exchange(true)) {
      tools::logger()->info("[Gimbal] First q received.");
      if (adapter_.enabled) {
        tools::logger()->info(
          "[Gimbal] Protocol adapter enabled: rx(yaw={}, pitch={}, roll={}), tx(yaw={}, pitch={})",
          adapter_.rx_yaw_sign, adapter_.rx_pitch_sign, adapter_.rx_roll_sign, adapter_.tx_yaw_sign,
          adapter_.tx_pitch_sign);
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_q_ = q;

    const auto rx_yaw_sign = adapter_.enabled ? adapter_.rx_yaw_sign : 1;
    const auto rx_pitch_sign = adapter_.enabled ? adapter_.rx_pitch_sign : 1;
    state_.yaw = rx_yaw_sign * rx_data_.yaw;
    state_.yaw_vel = rx_yaw_sign * rx_data_.yaw_vel;
    state_.pitch = rx_pitch_sign * rx_data_.pitch;
    state_.pitch_vel = rx_pitch_sign * rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    // 更新CBoard兼容的公共成员变量
    bullet_speed = rx_data_.bullet_speed;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        mode_cboard = idle;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        mode_cboard = auto_aim;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        mode_cboard = small_buff;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        mode_cboard = big_buff;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        mode_cboard = idle;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();
      serial_.setBaudrate(115200);
      serial_.setTimeout(serial::Timeout::max(), 1000, 0, 1000, 0);
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
