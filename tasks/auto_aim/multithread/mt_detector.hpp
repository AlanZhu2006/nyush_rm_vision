#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#ifdef USE_TENSORRT
#include "tools/tensorrt_engine.hpp"
#endif
#include <tuple>
#include <vector>

#include "tasks/auto_aim/yolos/yolov5.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{

class MultiThreadDetector
{
public:
  MultiThreadDetector(const std::string & config_path, bool debug = false);

  void push(cv::Mat img, std::chrono::steady_clock::time_point t);

  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();  //暂时不支持yolov8

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();

private:
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  std::string device_;
  std::string backend_;
#ifdef USE_TENSORRT
  std::unique_ptr<tools::TrtEngine> trt_engine_;
#endif
  YOLO yolo_;

  tools::ThreadSafeQueue<
    std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest>, true>
    queue_{6, [] { tools::logger()->debug("[MultiThreadDetector] queue is full, drop oldest"); }};

  tools::ThreadSafeQueue<
    std::tuple<cv::Mat, std::chrono::steady_clock::time_point, std::vector<float>>, true>
    trt_queue_{6, [] { tools::logger()->debug("[MultiThreadDetector] trt queue is full, drop oldest"); }};
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP
