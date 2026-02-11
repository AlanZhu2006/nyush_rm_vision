#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? | | Print help message}"
  "{@config-path   | | Path to YAML config file}";

using namespace std::chrono_literals;

namespace
{
Eigen::Vector3d quaternion_to_rm_euler_deg(const Eigen::Quaterniond & q)
{
  const double w = q.w();
  const double x = q.x();
  const double y = q.y();
  const double z = q.z();

  // Keep the same convention as nyush-rm-control/modules/imu/ins_task.c::QuaternionToEularAngle:
  // yaw(z), pitch(x), roll(y), all in degrees.
  const double yaw = std::atan2(2.0 * (w * z + x * y), 2.0 * (w * w + x * x) - 1.0);
  const double pitch = std::atan2(2.0 * (w * x + y * z), 2.0 * (w * w + z * z) - 1.0);
  const double roll = std::asin(std::clamp(2.0 * (w * y - x * z), -1.0, 1.0));

  constexpr double RAD2DEG = 180.0 / M_PI;
  return {yaw * RAD2DEG, pitch * RAD2DEG, roll * RAD2DEG};
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  io::Gimbal gimbal(config_path);

  fmt::print("\n");
  fmt::print("=================================================================\n");
  fmt::print("      IMU Communication Test - Vision PC <-> Gimbal Board\n");
  fmt::print("=================================================================\n");
  fmt::print("\n");
  fmt::print("Test Contents:\n");
  fmt::print("  1. Receive IMU data from gimbal board (quaternion, euler, state)\n");
  fmt::print("  2. Send control commands to gimbal board\n");
  fmt::print("  3. Monitor communication quality (frequency, latency)\n");
  fmt::print("\n");
  fmt::print("Press 'q' to exit\n");
  fmt::print("-----------------------------------------------------------------\n\n");

  auto t0 = std::chrono::steady_clock::now();
  auto last_t = t0;
  int frame_count = 0;
  double total_dt = 0.0;
  double max_dt = 0.0;
  double min_dt = 1000.0;

  // Send command counter
  int send_count = 0;
  auto last_mode = gimbal.mode();

  while (!exiter.exit()) {
    auto t = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(t, last_t);
    last_t = t;

    // ========== 1. Receive data from gimbal board ==========

    // Get IMU quaternion data
    auto q = gimbal.q(t);

    // Get gimbal state
    auto state = gimbal.state();
    auto mode = gimbal.mode();

    // Convert quaternion using the same Euler convention as lower machine firmware.
    // This avoids pitch/roll name confusion when checking protocol consistency.
    auto ypr_deg = quaternion_to_rm_euler_deg(q);
    double yaw_deg = ypr_deg[0];
    double pitch_deg = ypr_deg[1];
    double roll_deg = ypr_deg[2];

    // Statistics for communication quality
    frame_count++;
    total_dt += dt;
    if (dt > max_dt) max_dt = dt;
    if (dt < min_dt) min_dt = dt;

    // ========== 2. Send control commands to gimbal board ==========

    // Send simple control command (maintain current pose, no fire)
    gimbal.send(
      true,   // control = true
      false,  // fire = false
      0.0,    // yaw = 0
      0.0,    // yaw_vel = 0
      0.0,    // yaw_acc = 0
      0.0,    // pitch = 0
      0.0,    // pitch_vel = 0
      0.0     // pitch_acc = 0
    );
    send_count++;

    // ========== 3. Print data ==========

    // Print detailed data every 10 frames
    if (frame_count % 10 == 0) {
      auto elapsed = tools::delta_time(t, t0);
      double avg_fps = frame_count / elapsed;
      double avg_dt = total_dt / frame_count;

      fmt::print("\033[2J\033[H");  // Clear screen

      fmt::print("=================================================================\n");
      fmt::print("              IMU Communication Test - Real-time Data\n");
      fmt::print("=================================================================\n");
      fmt::print("Runtime: {:.2f}s  |  Frames: {}  |  Frequency: {:.1f} Hz\n",
                 elapsed, frame_count, avg_fps);
      fmt::print("=================================================================\n\n");

      // Data received from gimbal board
      fmt::print("[RECEIVE] Gimbal Board -> Vision PC\n");
      fmt::print("-----------------------------------------------------------------\n");

      // Gimbal mode
      fmt::print("  Mode: {}\n\n", gimbal.str(mode));

      // IMU pose data
      fmt::print("  IMU Pose Data:\n");

      // Quaternion raw data
      fmt::print("    Quaternion:\n");
      fmt::print("      w = {: 7.4f}\n", q.w());
      fmt::print("      x = {: 7.4f}\n", q.x());
      fmt::print("      y = {: 7.4f}\n", q.y());
      fmt::print("      z = {: 7.4f}\n", q.z());

      fmt::print("\n");

      // Euler angles (degrees)
      fmt::print("    Euler Angles:\n");
      fmt::print("      Yaw   = {: 8.2f}°\n", yaw_deg);
      fmt::print("      Pitch = {: 8.2f}°\n", pitch_deg);
      fmt::print("      Roll  = {: 8.2f}°\n", roll_deg);

      fmt::print("\n");

      fmt::print("  Gimbal Encoder Data:\n");
      fmt::print("    Yaw   Position: {: 8.2f}°  Velocity: {: 7.2f}°/s\n",
                 state.yaw * 180.0 / M_PI, state.yaw_vel * 180.0 / M_PI);
      fmt::print("    Pitch Position: {: 8.2f}°  Velocity: {: 7.2f}°/s\n",
                 state.pitch * 180.0 / M_PI, state.pitch_vel * 180.0 / M_PI);

      fmt::print("\n");

      fmt::print("  Shooting Data:\n");
      fmt::print("    Bullet Speed: {: 6.2f} m/s\n", state.bullet_speed);
      fmt::print("    Shot Count:   {}\n", state.bullet_count);

      fmt::print("\n");

      // Data sent to gimbal board
      fmt::print("[SEND] Vision PC -> Gimbal Board\n");
      fmt::print("-----------------------------------------------------------------\n");
      fmt::print("  Commands Sent: {}\n", send_count);
      fmt::print("  Control Mode:  Enabled (control = true)\n");
      fmt::print("  Fire Command:  Disabled (fire = false)\n");
      fmt::print("  Yaw   Command: 0.00° (vel: 0.00°/s, acc: 0.00°/s²)\n");
      fmt::print("  Pitch Command: 0.00° (vel: 0.00°/s, acc: 0.00°/s²)\n");

      fmt::print("\n");

      // Communication quality statistics
      fmt::print("[QUALITY] Communication Statistics\n");
      fmt::print("-----------------------------------------------------------------\n");
      fmt::print("  Average Frequency: {: 6.2f} Hz\n", avg_fps);
      fmt::print("  Average Latency:   {: 6.2f} ms\n", avg_dt * 1000);
      fmt::print("  Max Latency:       {: 6.2f} ms\n", max_dt * 1000);
      fmt::print("  Min Latency:       {: 6.2f} ms\n", min_dt * 1000);

      // Latency warnings
      if (avg_dt > 0.020) {  // Over 20ms
        fmt::print("  ⚠ WARNING: High latency detected!\n");
      } else if (avg_fps < 50) {  // Below 50Hz
        fmt::print("  ⚠ WARNING: Low frequency detected!\n");
      } else {
        fmt::print("  ✓ Communication status: Good\n");
      }

      fmt::print("\n");

      // Mode change detection
      if (mode != last_mode) {
        fmt::print("🔔 Mode changed: {} -> {}\n\n",
                   gimbal.str(last_mode), gimbal.str(mode));
        last_mode = mode;
      }

      fmt::print("Press 'q' to exit...\n");
    }

    // 检查键盘输入
    auto key = cv::waitKey(1);
    if (key == 'q') break;

    std::this_thread::sleep_for(9ms);  // ~100Hz
  }

  // Test finished, send stop command
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  fmt::print("\n\n");
  fmt::print("=================================================================\n");
  fmt::print("                   Test Finished - Summary Report\n");
  fmt::print("=================================================================\n");

  auto total_time = tools::delta_time(last_t, t0);
  double avg_fps = frame_count / total_time;
  double avg_dt = total_dt / frame_count;

  fmt::print("  Total Runtime:      {:.2f} s\n", total_time);
  fmt::print("  Total Frames:       {}\n", frame_count);
  fmt::print("  Average Frequency:  {:.2f} Hz\n", avg_fps);
  fmt::print("  Average Latency:    {:.2f} ms\n", avg_dt * 1000);
  fmt::print("  Max Latency:        {:.2f} ms\n", max_dt * 1000);
  fmt::print("  Min Latency:        {:.2f} ms\n", min_dt * 1000);
  fmt::print("\n");

  // Communication quality assessment
  fmt::print("  Communication Quality Assessment:\n");
  if (avg_fps > 80 && avg_dt < 0.015) {
    fmt::print("    ✓✓✓ Excellent - Stable and reliable\n");
  } else if (avg_fps > 50 && avg_dt < 0.025) {
    fmt::print("    ✓✓  Good - Generally normal\n");
  } else if (avg_fps > 30 && avg_dt < 0.040) {
    fmt::print("    ✓   Fair - Barely acceptable\n");
  } else {
    fmt::print("    ✗   Poor - Check serial connection and configuration\n");
  }

  fmt::print("\n");
  fmt::print("=================================================================\n");
  fmt::print("\n");

  return 0;
}
