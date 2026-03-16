#ifndef AUTO_AIM__ASYNC_DETECTOR_HPP
#define AUTO_AIM__ASYNC_DETECTOR_HPP

#include <chrono>
#include <future>
#include <list>
#include <opencv2/opencv.hpp>
#include <string>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

/**
 * @brief Asynchronous YOLO detector wrapper for parallel inference
 * 
 * This class allows YOLO inference to run in parallel with the main
 * processing loop, reducing effective latency by overlapping computation.
 * 
 * Usage pattern:
 *   1. submit() current frame for async detection
 *   2. get_result() retrieves results from previous frame
 *   3. Continue with tracking/aiming while next frame is being processed
 */
class AsyncDetector
{
public:
  /**
   * @brief Constructor
   * @param config_path Path to YAML configuration file
   * @param debug Enable debug visualization (default: false)
   */
  explicit AsyncDetector(const std::string & config_path, bool debug = false)
  : detector_(config_path, debug), frame_count_(0)
  {
  }

  /**
   * @brief Submit an image for asynchronous detection
   * @param img Input image (will be copied for async processing)
   * @param t Timestamp of the frame
   * 
   * Note: Captures img by value in lambda to ensure thread safety
   */
  void submit(const cv::Mat & img, std::chrono::steady_clock::time_point t)
  {
    // Clone the image to avoid race conditions (mat data is shared by default)
    cv::Mat img_copy = img.clone();
    int current_frame = frame_count_++;

    future_ = std::async(
      std::launch::async, [this, img_copy, current_frame]() {
        return detector_.detect(img_copy, current_frame);
      });

    last_timestamp_ = t;
  }

  /**
   * @brief Get detection results from the previously submitted frame
   * @return Tuple of (armors, timestamp). Returns empty list if no valid result.
   * 
   * This call blocks until the async inference completes.
   */
  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> get_result()
  {
    if (future_.valid()) {
      auto armors = future_.get();  // Block until inference completes
      return {std::move(armors), last_timestamp_};
    }
    return {{}, last_timestamp_};
  }

  /**
   * @brief Check if there's a pending inference
   * @return true if async inference is running
   */
  bool is_busy() const
  {
    return future_.valid() && future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
  }

private:
  std::future<std::list<Armor>> future_;
  auto_aim::YOLO detector_;
  std::chrono::steady_clock::time_point last_timestamp_;
  int frame_count_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ASYNC_DETECTOR_HPP