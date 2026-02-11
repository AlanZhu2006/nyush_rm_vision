#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                           | 输出命令行参数说明 }"
  "{@config-path   | configs/standard.yaml     | yaml配置文件的路径}"
  "{log-interval l | 10                        | 终端输出间隔帧数  }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);
  auto log_interval = cli.get<int>("log-interval");
  if (config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  io::Camera camera(config_path);
  io::Gimbal cboard(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_stamp = std::chrono::steady_clock::now();
  int frame_count = 0;

  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) break;

    auto q = cboard.imu_at(t - 1ms);
    auto mode = cboard.mode_cboard;

    solver.set_R_gimbal2world(q);

    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);
    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    if (mode != io::Mode::auto_aim) {
      command.control = false;
      command.shoot = false;
    }

    cboard.send(command);

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last_stamp);
    last_stamp = now;

    frame_count++;
    if (mode == io::Mode::auto_aim && frame_count % log_interval == 0) {
      tools::logger()->info(
        "cmd: control={} yaw={:.2f} pitch={:.2f} shoot={} | fps {:.2f}",
        command.control, command.yaw * 57.3, command.pitch * 57.3, command.shoot, 1.0 / dt);
    }

    tools::draw_text(
      img,
      fmt::format(
        "mode:{}", io::MODES[mode]),
      {10, 30}, {255, 255, 255});

    tools::draw_text(
      img,
      fmt::format(
        "cmd: {} yaw:{:.2f} pitch:{:.2f} shoot:{}",
        command.control, command.yaw * 57.3, command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    cv::imshow("auto_aim_camera_test", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
