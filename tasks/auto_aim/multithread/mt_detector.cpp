#include "mt_detector.hpp"

#include <opencv2/dnn.hpp>
#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  if (yaml["backend"]) {
    backend_ = yaml["backend"].as<std::string>();
  } else {
    backend_ = "openvino";
  }
  if (backend_ == "tensorrt") {
    auto trt_key = yolo_name + std::string("_trt_engine_path");
    if (yaml[trt_key]) {
      model_path = yaml[trt_key].as<std::string>();
    }
  }

  if (backend_ == "tensorrt") {
#ifdef USE_TENSORRT
    trt_engine_ = std::make_unique<tools::TrtEngine>(model_path, 3, 640, 640);
#else
    throw std::runtime_error("TensorRT backend requested but USE_TENSORRT is OFF");
#endif
  } else {
    auto model = core_.read_model(model_path);
    ov::preprocess::PrePostProcessor ppp(model);
    auto & input = ppp.input();

    input.tensor()
      .set_element_type(ov::element::u8)
      .set_shape({1, 640, 640, 3})  // TODO
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);

    input.model().set_layout("NCHW");

    input.preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
      .scale(255.0);

    model = ppp.build();
    compiled_model_ = core_.compile_model(
      model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  }

  tools::logger()->info("[MultiThreadDetector] initialized !");
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(img, input(roi), {w, h});

  if (backend_ == "tensorrt") {
#ifdef USE_TENSORRT
    cv::Mat blob = cv::dnn::blobFromImage(
      input, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false, CV_32F);
    auto output_data = trt_engine_->infer(blob.ptr<float>(), blob.total());
    trt_queue_.push({img.clone(), t, std::move(output_data)});
#else
    throw std::runtime_error("TensorRT backend requested but USE_TENSORRT is OFF");
#endif
    return;
  }

  auto infer_request = compiled_model_.create_infer_request();
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  infer_request.set_input_tensor(input_tensor);
  infer_request.start_async();
  queue_.push({img.clone(), t, std::move(infer_request)});
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  if (backend_ == "tensorrt") {
#ifdef USE_TENSORRT
    auto [img, t, output_data] = trt_queue_.pop();
    auto shape = trt_engine_->output_shape();
    if (shape.size() != 3) {
      throw std::runtime_error("Unexpected TensorRT output shape");
    }
    cv::Mat output(shape[1], shape[2], CV_32F, output_data.data());
    auto x_scale = static_cast<double>(640) / img.rows;
    auto y_scale = static_cast<double>(640) / img.cols;
    auto scale = std::min(x_scale, y_scale);
    auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI
    return {std::move(armors), t};
#else
    throw std::runtime_error("TensorRT backend requested but USE_TENSORRT is OFF");
#endif
  }

  auto [img, t, infer_request] = queue_.pop();
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI

  return {std::move(armors), t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  if (backend_ == "tensorrt") {
#ifdef USE_TENSORRT
    auto [img, t, output_data] = trt_queue_.pop();
    auto shape = trt_engine_->output_shape();
    if (shape.size() != 3) {
      throw std::runtime_error("Unexpected TensorRT output shape");
    }
    cv::Mat output(shape[1], shape[2], CV_32F, output_data.data());
    auto x_scale = static_cast<double>(640) / img.rows;
    auto y_scale = static_cast<double>(640) / img.cols;
    auto scale = std::min(x_scale, y_scale);
    auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI
    return {img, std::move(armors), t};
#else
    throw std::runtime_error("TensorRT backend requested but USE_TENSORRT is OFF");
#endif
  }

  auto [img, t, infer_request] = queue_.pop();
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI

  return {img, std::move(armors), t};
}

}  // namespace multithread

}  // namespace auto_aim
