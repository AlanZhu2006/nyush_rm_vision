#include <fmt/core.h>

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/web/web_display_worker.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                           | 输出命令行参数说明 }"
  "{@config-path   | configs/standard.yaml     | yaml配置文件的路径}"
  "{log-interval l | 10                        | 终端输出间隔帧数  }"
  "{send s         |                           | 发送控制到下位机  }"
  "{d display      |                           | 显示检测调试画面  }"
  "{web w          |                           | 使用Web界面显示(localhost:8080)  }"
  "{web-port       | 8080                      | Web服务器端口号   }";

namespace
{
constexpr int kTxHoldFrames = 0;
constexpr int kTxWarmupFrames = 1;
constexpr double kTxMaxYawRateRadS = 4.0;
constexpr double kTxMaxPitchRateRadS = 2.0;
constexpr double kTxMinDtS = 1e-3;
constexpr double kTxMaxCaptureAgeS = 0.20;
constexpr int kX11ForwardDisplayMaxWidth = 480;

bool has_desktop_session()
{
  return std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
}

void try_open_web_display_in_browser(int web_port)
{
  if (!has_desktop_session()) {
    tools::logger()->info(
      "desktop session not detected, open http://127.0.0.1:{} manually", web_port);
    return;
  }

  const auto url = fmt::format("http://127.0.0.1:{}", web_port);
  std::thread([url]() {
    std::this_thread::sleep_for(300ms);
#if defined(_WIN32)
    const auto command = fmt::format("cmd.exe /C start \"\" \"{}\"", url);
#elif defined(__APPLE__)
    const auto command = fmt::format("open \"{}\" >/dev/null 2>&1", url);
#else
    const auto command = fmt::format("xdg-open \"{}\" >/dev/null 2>&1", url);
#endif
    if (std::system(command.c_str()) != 0) {
      tools::logger()->warn("failed to open browser automatically, open {} manually", url);
    }
  }).detach();
}

bool is_likely_x11_forwarding()
{
  const char * display_env = std::getenv("DISPLAY");
  if (display_env == nullptr) return false;
  std::string display(display_env);

  const bool has_ssh = std::getenv("SSH_CONNECTION") != nullptr ||
                       std::getenv("SSH_CLIENT") != nullptr || std::getenv("SSH_TTY") != nullptr;
  const bool local_display = display.rfind(":0", 0) == 0 || display.rfind(":1", 0) == 0;
  const bool forwarded_display = display.rfind("localhost:", 0) == 0 || display.rfind("127.0.0.1:", 0) == 0 ||
                                 (!local_display && display.find(':') != std::string::npos);

  return has_ssh && forwarded_display;
}

std::filesystem::path make_detect_log_path()
{
  std::filesystem::path log_dir = "logs";
  std::error_code ec;
  std::filesystem::create_directories(log_dir, ec);

  auto now = std::chrono::system_clock::now();
  auto time_now = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
#ifdef _WIN32
  localtime_s(&local_tm, &time_now);
#else
  localtime_r(&time_now, &local_tm);
#endif

  std::ostringstream name;
  name << "detect_rx_tx_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << ".csv";
  return log_dir / name.str();
}

class DisplayWorker
{
 public:
  explicit DisplayWorker(std::string window_name, bool reduce_resolution)
    : window_name_(std::move(window_name)), reduce_resolution_(reduce_resolution)
  {
  }

  ~DisplayWorker()
  {
    stop();
  }

  void start()
  {
    worker_ = std::thread([this]() { run(); });
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_requested_ = true;
    }
    cond_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void submit(const cv::Mat & frame)
  {
    if (failed_.load()) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frame.copyTo(latest_frame_);
      has_frame_ = true;
    }
    cond_.notify_one();
  }

  bool should_exit() const
  {
    return exit_requested_.load();
  }

  bool failed() const
  {
    return failed_.load();
  }

 private:
  void run()
  {
    try {
      while (true) {
        cv::Mat frame;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          cond_.wait_for(lock, 5ms, [this]() { return stop_requested_ || has_frame_; });
          if (stop_requested_) break;
          if (has_frame_) {
            latest_frame_.copyTo(frame);
            has_frame_ = false;
          }
        }

        if (!frame.empty()) {
          cv::Mat shown_frame = frame;
          if (reduce_resolution_ && frame.cols > kX11ForwardDisplayMaxWidth) {
            const double scale =
              static_cast<double>(kX11ForwardDisplayMaxWidth) / static_cast<double>(frame.cols);
            const int target_w = std::max(1, static_cast<int>(frame.cols * scale));
            const int target_h = std::max(1, static_cast<int>(frame.rows * scale));
            cv::resize(frame, shown_frame, cv::Size(target_w, target_h), 0.0, 0.0, cv::INTER_AREA);
          }
          cv::imshow(window_name_, shown_frame);
        }
        if (cv::waitKey(1) == 'q') {
          exit_requested_.store(true);
          break;
        }
      }
      cv::destroyWindow(window_name_);
    } catch (const cv::Exception & e) {
      tools::logger()->warn("disable display due to OpenCV GUI error: {}", e.what());
      failed_.store(true);
    }
  }

  std::string window_name_;
  bool reduce_resolution_ = false;
  std::thread worker_;
  mutable std::mutex mutex_;
  std::condition_variable cond_;
  cv::Mat latest_frame_;
  bool has_frame_ = false;
  bool stop_requested_ = false;
  std::atomic<bool> exit_requested_{false};
  std::atomic<bool> failed_{false};
};
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);
  auto log_interval = cli.get<int>("log-interval");
  auto send_to_gimbal = cli.has("send");
  auto display = cli.has("display");
  auto use_web = cli.has("web");
  auto web_port = cli.get<int>("web-port");
  if (config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // Web display takes precedence over X11 display
  if (use_web && display) {
    tools::logger()->warn("Both --web and --display specified, using web display");
    display = false;
  }

  if (display && std::getenv("DISPLAY") == nullptr) {
    tools::logger()->warn("display requested but DISPLAY is unset, fallback to headless mode");
    display = false;
  }

  tools::logger()->info("auto_aim_camera_test started, send={}, display_mode={}", 
                        send_to_gimbal, use_web ? "web" : (display ? "x11" : "headless"));

  tools::Exiter exiter;

  auto detect_log_path = make_detect_log_path();
  std::ofstream detect_log(detect_log_path, std::ios::out | std::ios::trunc);
  if (!detect_log.is_open()) {
    tools::logger()->warn("failed to open detect rx/tx log file: {}", detect_log_path.string());
  } else {
    detect_log << std::fixed << std::setprecision(6);
    detect_log
      << "frame,elapsed_s,capture_age_s,armors,mode,tx_enable,tx_raw_control,tx_raw_shoot,"
      << "tx_raw_yaw_rad,tx_raw_pitch_rad,tx_control,tx_shoot,tx_yaw_rad,tx_pitch_rad,"
      << "tx_horizon_distance_m,tx_hold_applied,tx_slew_applied,tx_startup_sync_applied,"
      << "tx_raw_streak,rx_yaw_rad,rx_pitch_rad,"
      << "rx_yaw_vel_rad_s,"
      << "rx_pitch_vel_rad_s,rx_bullet_speed_mps,rx_bullet_count\n";
    tools::logger()->info("detect rx/tx log file: {}", detect_log_path.string());
  }

  io::Camera camera(config_path);
  io::Gimbal cboard(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_stamp = std::chrono::steady_clock::now();
  auto test_start = last_stamp;
  auto last_tx_stamp = last_stamp;
  io::Command last_sent_command{false, false, 0.0, 0.0, 0.0};
  bool has_last_sent_command = false;
  int hold_frames_left = 0;
  int raw_control_streak = 0;
  int frame_count = 0;
  std::unique_ptr<DisplayWorker> display_worker;
  std::unique_ptr<tools::WebDisplayWorker> web_display_worker;

  if (use_web) {
    // Web display: title, port, JPEG quality (60 for faster encoding), max width (480 for smooth streaming)
    web_display_worker = std::make_unique<tools::WebDisplayWorker>("Auto Aim Camera Test", web_port, 60, 480);
    web_display_worker->start();
    tools::logger()->info("Web display started on http://localhost:{}", web_port);
    try_open_web_display_in_browser(web_port);
  } else if (display) {
    const bool x11_forwarding = is_likely_x11_forwarding();
    if (x11_forwarding) {
      tools::logger()->info(
        "X11 forwarding detected, display will be downscaled to width <= {} for smoother GUI",
        kX11ForwardDisplayMaxWidth);
    }
    display_worker = std::make_unique<DisplayWorker>("auto_aim_camera_test", x11_forwarding);
    display_worker->start();
  }

  while (!exiter.exit() && !(display_worker && display_worker->should_exit()) && 
         !(web_display_worker && web_display_worker->failed())) {
    camera.read(img, t);
    if (img.empty()) break;

    auto q = cboard.imu_at(t - 1ms);
    auto mode = cboard.mode_cboard;
    auto gimbal_state = cboard.state();
    auto control_now = std::chrono::steady_clock::now();
    auto capture_age_s = tools::delta_time(control_now, t);
    bool stale_capture = capture_age_s > kTxMaxCaptureAgeS;

    solver.set_R_gimbal2world(q);
    auto gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);
    std::list<auto_aim::Target> targets;
    if (!stale_capture) {
      targets = tracker.track(armors, t);
    } else {
      armors.clear();
    }
    auto raw_command = aimer.aim(targets, t, cboard.bullet_speed);
    if (stale_capture || armors.empty()) {
      raw_command.control = false;
      raw_command.shoot = false;
    }
    if (stale_capture) {
      raw_command.yaw = gimbal_state.yaw;
      raw_command.pitch = gimbal_state.pitch;
    }
    auto command = raw_command;
    bool hold_applied = false;
    bool slew_applied = false;
    bool startup_sync_applied = false;

    if (mode != io::Mode::auto_aim) {
      raw_command.control = false;
      raw_command.shoot = false;
      command = raw_command;
      hold_frames_left = 0;
      raw_control_streak = 0;
      has_last_sent_command = false;
    } else {
      if (raw_command.control) {
        raw_control_streak++;
      } else {
        raw_control_streak = 0;
      }

      const bool raw_control_ready = raw_control_streak >= kTxWarmupFrames;
      const bool effective_control = raw_command.control && raw_control_ready;

      if (effective_control) {
        hold_frames_left = kTxHoldFrames;
      } else if (has_last_sent_command && last_sent_command.control && hold_frames_left > 0) {
        command = last_sent_command;
        command.control = true;
        command.shoot = false;
        hold_frames_left--;
        hold_applied = true;
      } else {
        command.control = false;
        command.shoot = false;
        hold_frames_left = 0;
      }

      if (effective_control && !hold_applied) {
        command.control = true;
        command.shoot = raw_command.shoot;
        command.yaw = raw_command.yaw;
        command.pitch = raw_command.pitch;
      }

      if (command.control && has_last_sent_command && last_sent_command.control && !hold_applied) {
        auto tx_now = std::chrono::steady_clock::now();
        auto tx_dt = tools::delta_time(tx_now, last_tx_stamp);
        if (tx_dt < kTxMinDtS) tx_dt = kTxMinDtS;

        const auto max_yaw_step = kTxMaxYawRateRadS * tx_dt;
        const auto max_pitch_step = kTxMaxPitchRateRadS * tx_dt;

        const auto yaw_delta = tools::limit_rad(command.yaw - last_sent_command.yaw);
        if (yaw_delta > max_yaw_step) {
          command.yaw = tools::limit_rad(last_sent_command.yaw + max_yaw_step);
          slew_applied = true;
        } else if (yaw_delta < -max_yaw_step) {
          command.yaw = tools::limit_rad(last_sent_command.yaw - max_yaw_step);
          slew_applied = true;
        }

        const auto pitch_delta = command.pitch - last_sent_command.pitch;
        if (pitch_delta > max_pitch_step) {
          command.pitch = last_sent_command.pitch + max_pitch_step;
          slew_applied = true;
        } else if (pitch_delta < -max_pitch_step) {
          command.pitch = last_sent_command.pitch - max_pitch_step;
          slew_applied = true;
        }
      }

      if (!command.control) {
        command.yaw = gimbal_state.yaw;
        command.pitch = gimbal_state.pitch;
      }
    }

    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    if (send_to_gimbal) {
      float tx_yaw_vel = 0.0f;
      float tx_pitch_vel = 0.0f;
      if (command.control && has_last_sent_command && last_sent_command.control) {
        auto tx_now = std::chrono::steady_clock::now();
        auto tx_dt = tools::delta_time(tx_now, last_tx_stamp);
        if (tx_dt < kTxMinDtS) tx_dt = kTxMinDtS;
        tx_yaw_vel =
          static_cast<float>(tools::limit_rad(command.yaw - last_sent_command.yaw) / tx_dt);
        tx_pitch_vel = static_cast<float>((command.pitch - last_sent_command.pitch) / tx_dt);
      }
      cboard.send(
        command.control, command.shoot, static_cast<float>(command.yaw), tx_yaw_vel, 0.0f,
        static_cast<float>(command.pitch), tx_pitch_vel, 0.0f);
    }

    if (command.control) {
      last_sent_command = command;
      has_last_sent_command = true;
      last_tx_stamp = std::chrono::steady_clock::now();
    }

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last_stamp);
    last_stamp = now;
    auto fps = dt > 1e-6 ? 1.0 / dt : 0.0;

    frame_count++;
    if (detect_log.is_open()) {
      auto elapsed_s = tools::delta_time(now, test_start);
      detect_log << frame_count << ',' << elapsed_s << ',' << capture_age_s << ',' << armors.size()
                 << ',' << static_cast<int>(mode) << ',' << (send_to_gimbal ? 1 : 0) << ','
                 << (raw_command.control ? 1 : 0) << ',' << (raw_command.shoot ? 1 : 0) << ','
                 << raw_command.yaw << ',' << raw_command.pitch << ','
                 << (command.control ? 1 : 0) << ',' << (command.shoot ? 1 : 0) << ','
                 << command.yaw << ',' << command.pitch << ',' << command.horizon_distance << ','
                 << (hold_applied ? 1 : 0) << ',' << (slew_applied ? 1 : 0) << ','
                 << (startup_sync_applied ? 1 : 0) << ',' << raw_control_streak << ','
                 << gimbal_state.yaw << ',' << gimbal_state.pitch << ',' << gimbal_state.yaw_vel
                 << ',' << gimbal_state.pitch_vel << ',' << gimbal_state.bullet_speed << ','
                 << gimbal_state.bullet_count << '\n';
      if ((frame_count % 50) == 0) {
        detect_log.flush();
      }
    }

    if (mode == io::Mode::auto_aim && frame_count % log_interval == 0) {
      tools::logger()->info(
        "armors:{} tx:{} stale={} raw_ctl={} streak={} raw_yaw={:.2f} | sent_ctl={} yaw={:.2f} "
        "pitch={:.2f} hold={} slew={} sync={} shoot={} | fps {:.2f}",
        armors.size(), send_to_gimbal, stale_capture,
        raw_command.control, raw_control_streak,
        raw_command.yaw * 57.3, command.control, command.yaw * 57.3, command.pitch * 57.3,
        hold_applied, slew_applied, startup_sync_applied, command.shoot, fps);
    }

    // Draw armor detection boxes
    for (const auto & armor : armors) {
      tools::draw_points(img, armor.points, {0, 255, 0});
      // Only draw text labels in X11 display mode, not in web mode
      if (!use_web) {
        tools::draw_text(
          img, fmt::format("{} {:.2f}", auto_aim::ARMOR_NAMES[armor.name], armor.confidence),
          armor.center, {0, 255, 0}, 0.6, 2);
      }
    }

    // Draw overlays only for X11 display mode, not for web (web shows data separately)
    if (!use_web) {
      tools::draw_text(
        img,
        fmt::format(
          "mode:{} armors:{} tx:{} fps:{:.2f}", io::MODES[mode], armors.size(), send_to_gimbal, fps),
        {10, 30}, {255, 255, 255});

      tools::draw_text(
        img,
        fmt::format(
          "tx_sent(rad): tx:{} ctl:{} yaw:{:.4f} pitch:{:.4f} hold:{} slew:{} shoot:{}",
          send_to_gimbal, command.control, command.yaw, command.pitch, hold_applied, slew_applied,
          command.shoot),
        {10, 60}, {154, 50, 205});

      tools::draw_text(
        img,
        fmt::format(
          "tx_raw(rad): ctl:{} yaw:{:.4f} pitch:{:.4f}", raw_command.control, raw_command.yaw,
          raw_command.pitch),
        {10, 90}, {180, 120, 255});

      tools::draw_text(
        img,
        fmt::format(
          "rx(rad): yaw:{:.2f} pitch:{:.2f} yaw_v:{:.2f} pitch_v:{:.2f}",
          gimbal_state.yaw, gimbal_state.pitch, gimbal_state.yaw_vel, gimbal_state.pitch_vel),
        {10, 120}, {255, 255, 0});

      tools::draw_text(
        img,
        fmt::format(
          "rx: bullet_speed:{:.2f} bullet_count:{}",
          gimbal_state.bullet_speed, gimbal_state.bullet_count),
        {10, 150}, {255, 255, 0});
    }

    // Generate metadata JSON for web display
    std::string metadata_json;
    if (web_display_worker) {
      metadata_json = fmt::format(
        "{{"
        "\"FPS\": \"{:.2f}\","
        "\"Mode\": \"{}\","
        "\"Armors Detected\": \"{}\","
        "\"TX Enabled\": \"{}\","
        "\"TX Control\": \"{}\","
        "\"TX Shoot\": \"{}\","
        "\"TX Yaw (deg)\": \"{:.2f}\","
        "\"TX Pitch (deg)\": \"{:.2f}\","
        "\"TX Hold Applied\": \"{}\","
        "\"TX Slew Applied\": \"{}\","
        "\"Raw Control\": \"{}\","
        "\"Raw Yaw (deg)\": \"{:.2f}\","
        "\"Raw Pitch (deg)\": \"{:.2f}\","
        "\"Raw Streak\": \"{}\","
        "\"RX Yaw (deg)\": \"{:.2f}\","
        "\"RX Pitch (deg)\": \"{:.2f}\","
        "\"RX Yaw Vel (deg/s)\": \"{:.2f}\","
        "\"RX Pitch Vel (deg/s)\": \"{:.2f}\","
        "\"RX Bullet Speed (m/s)\": \"{:.2f}\","
        "\"RX Bullet Count\": \"{}\","
        "\"Capture Age (ms)\": \"{:.1f}\","
        "\"Stale Capture\": \"{}\""
        "}}",
        fps,
        io::MODES[mode],
        armors.size(),
        send_to_gimbal ? "true" : "false",
        command.control ? "true" : "false",
        command.shoot ? "true" : "false",
        command.yaw * 57.295779513,
        command.pitch * 57.295779513,
        hold_applied ? "true" : "false",
        slew_applied ? "true" : "false",
        raw_command.control ? "true" : "false",
        raw_command.yaw * 57.295779513,
        raw_command.pitch * 57.295779513,
        raw_control_streak,
        gimbal_state.yaw * 57.295779513,
        gimbal_state.pitch * 57.295779513,
        gimbal_state.yaw_vel * 57.295779513,
        gimbal_state.pitch_vel * 57.295779513,
        gimbal_state.bullet_speed,
        gimbal_state.bullet_count,
        capture_age_s * 1000.0,
        stale_capture ? "true" : "false"
      );
    }

    if (web_display_worker) {
      web_display_worker->submit(img, metadata_json);
      if (web_display_worker->failed()) {
        web_display_worker->stop();
        web_display_worker.reset();
      }
    } else if (display_worker) {
      display_worker->submit(img);
      if (display_worker->failed()) {
        display_worker->stop();
        display_worker.reset();
      }
    }
  }

  if (web_display_worker) {
    web_display_worker->stop();
  }
  
  if (display_worker) {
    display_worker->stop();
  }

  return 0;
}
