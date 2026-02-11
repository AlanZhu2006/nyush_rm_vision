#ifndef TOOLS__TENSORRT_ENGINE_HPP
#define TOOLS__TENSORRT_ENGINE_HPP

#ifdef USE_TENSORRT

#include <NvInfer.h>
#include <NvInferVersion.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace tools
{
class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

template <typename T>
struct TrtDestroy
{
  void operator()(T * obj) const
  {
    if (!obj) {
      return;
    }
#if defined(NV_TENSORRT_MAJOR) && NV_TENSORRT_MAJOR >= 10
    delete obj;
#else
    obj->destroy();
#endif
  }
};

inline size_t trt_volume(const nvinfer1::Dims & dims)
{
  size_t v = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    v *= static_cast<size_t>(dims.d[i]);
  }
  return v;
}

inline size_t trt_type_size(nvinfer1::DataType type)
{
  switch (type) {
    case nvinfer1::DataType::kFLOAT:
      return 4;
    case nvinfer1::DataType::kHALF:
      return 2;
    case nvinfer1::DataType::kINT8:
      return 1;
    case nvinfer1::DataType::kINT32:
      return 4;
    case nvinfer1::DataType::kBOOL:
      return 1;
    default:
      return 0;
  }
}

class TrtEngine
{
public:
  TrtEngine(const std::string & engine_path, int input_c, int input_h, int input_w)
  : input_c_(input_c), input_h_(input_h), input_w_(input_w)
  {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("TensorRT engine not found: " + engine_path);
    }
    file.seekg(0, std::ifstream::end);
    size_t size = file.tellg();
    file.seekg(0, std::ifstream::beg);
    std::vector<char> data(size);
    file.read(data.data(), size);

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("Failed to create TensorRT runtime");
    }
    engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
    if (!engine_) {
      throw std::runtime_error("Failed to deserialize TensorRT engine");
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("Failed to create TensorRT execution context");
    }

#if defined(NV_TENSORRT_MAJOR) && NV_TENSORRT_MAJOR >= 10
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = name;
      } else if (output_name_.empty()) {
        output_name_ = name;
      }
    }

    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("TensorRT IO tensor names not found");
    }

    nvinfer1::Dims input_dims{};
    input_dims.nbDims = 4;
    input_dims.d[0] = 1;
    input_dims.d[1] = input_c_;
    input_dims.d[2] = input_h_;
    input_dims.d[3] = input_w_;
    if (!context_->setInputShape(input_name_.c_str(), input_dims)) {
      throw std::runtime_error("Failed to set TensorRT input dimensions");
    }

    input_dims_ = context_->getTensorShape(input_name_.c_str());
    output_dims_ = context_->getTensorShape(output_name_.c_str());
    input_type_ = engine_->getTensorDataType(input_name_.c_str());
    output_type_ = engine_->getTensorDataType(output_name_.c_str());
#else
    for (int i = 0; i < engine_->getNbBindings(); ++i) {
      if (engine_->bindingIsInput(i)) {
        input_index_ = i;
      } else if (output_index_ == -1) {
        output_index_ = i;
      }
    }

    if (input_index_ == -1 || output_index_ == -1) {
      throw std::runtime_error("TensorRT bindings not found");
    }

    if (!engine_->hasImplicitBatchDimension()) {
      nvinfer1::Dims input_dims{};
      input_dims.nbDims = 4;
      input_dims.d[0] = 1;
      input_dims.d[1] = input_c_;
      input_dims.d[2] = input_h_;
      input_dims.d[3] = input_w_;
      if (!context_->setBindingDimensions(input_index_, input_dims)) {
        throw std::runtime_error("Failed to set TensorRT input dimensions");
      }
    }

    input_dims_ = context_->getBindingDimensions(input_index_);
    output_dims_ = context_->getBindingDimensions(output_index_);
    input_type_ = engine_->getBindingDataType(input_index_);
    output_type_ = engine_->getBindingDataType(output_index_);
#endif

    input_size_ = trt_volume(input_dims_);
    output_size_ = trt_volume(output_dims_);

    if (cudaStreamCreate(&stream_) != cudaSuccess) {
      throw std::runtime_error("Failed to create CUDA stream");
    }

    cudaMalloc(&device_input_, input_size_ * trt_type_size(input_type_));
    cudaMalloc(&device_output_, output_size_ * trt_type_size(output_type_));
  }

  ~TrtEngine()
  {
    if (device_input_) {
      cudaFree(device_input_);
    }
    if (device_output_) {
      cudaFree(device_output_);
    }
    if (stream_) {
      cudaStreamDestroy(stream_);
    }
  }

  std::vector<int64_t> output_shape() const
  {
    std::vector<int64_t> shape(output_dims_.nbDims);
    for (int i = 0; i < output_dims_.nbDims; ++i) {
      shape[i] = output_dims_.d[i];
    }
    return shape;
  }

  size_t output_size() const
  {
    return output_size_;
  }

  std::vector<float> infer(const float * input, size_t input_size)
  {
    if (input_size != input_size_) {
      throw std::runtime_error("TensorRT input size mismatch");
    }

    if (input_type_ == nvinfer1::DataType::kFLOAT) {
      cudaMemcpyAsync(
        device_input_, input, input_size_ * sizeof(float), cudaMemcpyHostToDevice, stream_);
    } else if (input_type_ == nvinfer1::DataType::kHALF) {
      std::vector<__half> input_half(input_size_);
      for (size_t i = 0; i < input_size_; ++i) {
        input_half[i] = __float2half(input[i]);
      }
      cudaMemcpyAsync(
        device_input_, input_half.data(), input_size_ * sizeof(__half), cudaMemcpyHostToDevice,
        stream_);
    } else {
      throw std::runtime_error("Unsupported TensorRT input type");
    }

#if defined(NV_TENSORRT_MAJOR) && NV_TENSORRT_MAJOR >= 10
    if (!context_->setTensorAddress(input_name_.c_str(), device_input_)) {
      throw std::runtime_error("Failed to set TensorRT input address");
    }
    if (!context_->setTensorAddress(output_name_.c_str(), device_output_)) {
      throw std::runtime_error("Failed to set TensorRT output address");
    }
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TensorRT enqueue failed");
    }
#else
    void * bindings[2] = {device_input_, device_output_};
    if (!context_->enqueueV2(bindings, stream_, nullptr)) {
      throw std::runtime_error("TensorRT enqueue failed");
    }
#endif

    std::vector<float> output(output_size_);
    if (output_type_ == nvinfer1::DataType::kFLOAT) {
      cudaMemcpyAsync(
        output.data(), device_output_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost,
        stream_);
    } else if (output_type_ == nvinfer1::DataType::kHALF) {
      std::vector<__half> output_half(output_size_);
      cudaMemcpyAsync(
        output_half.data(), device_output_, output_size_ * sizeof(__half), cudaMemcpyDeviceToHost,
        stream_);
      cudaStreamSynchronize(stream_);
      for (size_t i = 0; i < output_size_; ++i) {
        output[i] = __half2float(output_half[i]);
      }
      return output;
    } else {
      throw std::runtime_error("Unsupported TensorRT output type");
    }

    cudaStreamSynchronize(stream_);
    return output;
  }

private:
  int input_c_;
  int input_h_;
  int input_w_;
  int input_index_ = -1;
  int output_index_ = -1;
  nvinfer1::Dims input_dims_{};
  nvinfer1::Dims output_dims_{};
  nvinfer1::DataType input_type_ = nvinfer1::DataType::kFLOAT;
  nvinfer1::DataType output_type_ = nvinfer1::DataType::kFLOAT;
  size_t input_size_ = 0;
  size_t output_size_ = 0;
  void * device_input_ = nullptr;
  void * device_output_ = nullptr;
  cudaStream_t stream_{};

  std::string input_name_;
  std::string output_name_;

  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime, TrtDestroy<nvinfer1::IRuntime>> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDestroy<nvinfer1::ICudaEngine>> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDestroy<nvinfer1::IExecutionContext>> context_;
};
}  // namespace tools

#endif  // USE_TENSORRT

#endif  // TOOLS__TENSORRT_ENGINE_HPP
