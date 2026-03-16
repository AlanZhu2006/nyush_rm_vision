/**
 * @file async_detector_usage_example.cpp
 * @brief Example showing how to use AsyncDetector to reduce latency
 * 
 * This demonstrates the "pipeline parallelism" approach where:
 * - Frame N is being captured/processed
 * - Frame N-1's YOLO inference runs in parallel
 * 
 * This reduces effective latency by overlapping computation stages.
 */

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolos/async_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | Print help message}"
  "{@config-path   | configs/standard3.yaml | Path to YAML config file }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  io::Gimbal cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::AsyncDetector async_detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  // First frame: bootstrap the pipeline
  camera.read(img, t);
  async_detector.submit(img, t);

  tools::logger()->info("Starting async detection pipeline...");

  while (!exiter.exit()) {
    // === STAGE 1: Capture new frame ===
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode_cboard;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // === STAGE 2: Get detection results from PREVIOUS frame ===
    // This blocks until async inference completes
    auto [armors, prev_t] = async_detector.get_result();

    // === STAGE 3: Submit CURRENT frame for async processing ===
    // This runs in parallel with stages 4-6
    async_detector.submit(img, t);

    // === STAGE 4-6: Process previous frame's detections ===
    solver.set_R_gimbal2world(q);
    auto targets = tracker.track(armors, prev_t);
    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    // === STAGE 7: Send command ===
    cboard.send(command);

    // Note: While we're doing stages 4-7, the next YOLO inference
    // is running in parallel, reducing total latency!
  }

  tools::logger()->info("Async detection pipeline stopped.");
  return 0;
}
