#include "camera.hpp"

#include <algorithm>
#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (yaml["camera_preprocess"] && yaml["camera_preprocess"]["enabled"] &&
      yaml["camera_preprocess"]["enabled"].as<bool>()) {
    auto preprocess = yaml["camera_preprocess"];
    preprocess_config_.enabled = true;
    preprocess_config_.crop_width = tools::read<int>(preprocess, "crop_width");
    preprocess_config_.crop_height = tools::read<int>(preprocess, "crop_height");
    preprocess_config_.output_width = tools::read<int>(preprocess, "output_width");
    preprocess_config_.output_height = tools::read<int>(preprocess, "output_height");
  }

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
  preprocess(img);
}

void Camera::preprocess(cv::Mat & img)
{
  if (!preprocess_config_.enabled || img.empty()) return;

  const int raw_width = img.cols;
  const int raw_height = img.rows;
  const int crop_width = preprocess_config_.crop_width;
  const int crop_height = preprocess_config_.crop_height;
  const int output_width = preprocess_config_.output_width;
  const int output_height = preprocess_config_.output_height;

  if (crop_width <= 0 || crop_height <= 0 || output_width <= 0 || output_height <= 0) {
    throw std::runtime_error("Invalid camera_preprocess dimensions");
  }

  if (crop_width > img.cols || crop_height > img.rows) {
    throw std::runtime_error(
      "camera_preprocess crop size exceeds raw frame size: raw=" + std::to_string(img.cols) + "x" +
      std::to_string(img.rows) + ", crop=" + std::to_string(crop_width) + "x" +
      std::to_string(crop_height));
  }

  const int offset_x = std::max(0, (img.cols - crop_width) / 2);
  const int offset_y = std::max(0, (img.rows - crop_height) / 2);
  const cv::Rect roi(offset_x, offset_y, crop_width, crop_height);

  cv::Mat cropped = img(roi);
  cv::Mat output;
  cv::resize(cropped, output, cv::Size(output_width, output_height));
  img = output;

  if (!preprocess_logged_) {
    tools::logger()->info(
      "camera preprocess enabled: raw={}x{}, crop={}x{}@({}, {}), output={}x{}",
      raw_width, raw_height, crop_width, crop_height, offset_x, offset_y, output_width,
      output_height);
  }
  preprocess_logged_ = true;
}

}  // namespace io
