#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

struct CameraPreprocessConfig
{
  bool enabled = false;
  int crop_width = 0;
  int crop_height = 0;
  int output_width = 0;
  int output_height = 0;
};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  void preprocess(cv::Mat & img);

  std::unique_ptr<CameraBase> camera_;
  CameraPreprocessConfig preprocess_config_;
  bool preprocess_logged_ = false;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
