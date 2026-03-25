#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "io/command.hpp"
#include "io/mode.hpp"
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) <= 64);

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw = 0.0F;
  float yaw_vel = 0.0F;
  float pitch = 0.0F;
  float pitch_vel = 0.0F;
  float bullet_speed = 0.0F;
  uint16_t bullet_count = 0;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  // 原有接口
  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

  // ===== CBoard兼容接口（新增）=====
  // 用于替换CBoard，使现有代码无需大量修改

  // CBoard风格的IMU获取接口（等价于q()）
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) {
    return q(timestamp);
  }

  // CBoard风格的命令发送接口
  void send(const Command & command) {
    send(command.control, command.shoot,
         command.yaw, 0, 0,  // yaw, yaw_vel, yaw_acc
         command.pitch, 0, 0);  // pitch, pitch_vel, pitch_acc
  }

  // 公共成员变量（CBoard兼容）
  double bullet_speed = 0.0;  // 实时更新
  Mode mode_cboard = idle;    // CBoard风格的mode

private:
  struct ProtocolAdapterConfig
  {
    bool enabled = false;
    int rx_yaw_sign = 1;
    int rx_pitch_sign = 1;
    int rx_roll_sign = 1;
    int tx_yaw_sign = 1;
    int tx_pitch_sign = 1;
  };

  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_{};
  VisionToGimbal tx_data_{};

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_{};
  ProtocolAdapterConfig adapter_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};
  Eigen::Quaterniond latest_q_ = Eigen::Quaterniond::Identity();
  std::atomic<bool> first_packet_received_{false};

  bool read(uint8_t * buffer, size_t size);
  VisionToGimbal adapt_tx(const VisionToGimbal & packet) const;
  Eigen::Quaterniond adapt_rx_quaternion(const Eigen::Quaterniond & q) const;
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
