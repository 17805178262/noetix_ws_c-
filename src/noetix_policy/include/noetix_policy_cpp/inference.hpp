// 中文说明：ONNX Runtime C++会话与预分配张量封装，直接调用原生推理，不嵌入Python。
// 会话预加载后可复用；每个策略实例独立持有输入输出张量包装，避免交接时共享可写动作缓冲。
// 默认CPU，GMT默认2线程、walk/motion各1线程；保留GMT显式1/2/4/auto与CUDA请求并严格报错。
#pragma once
#include <onnxruntime_cxx_api.h>

#include <memory>

#include "noetix_policy_cpp/core.hpp"
namespace noetix::policy {
struct Model {
  Model(const Spec& spec, const std::string& path);
  Ort::Session session{nullptr};
  std::vector<int64_t> output_shape;
  int threads = 1;
  std::string backend, report, sha256;
};
std::shared_ptr<Model> load_model(const Spec& spec, const std::string& path);
class Inference {
 public:
  Inference(const Spec& config, std::vector<std::vector<float>>& inputs,
            std::shared_ptr<Model> model);
  const float* run();
  const std::shared_ptr<Model> model;

 private:
  const Spec& config_;
  std::vector<std::vector<float>>& input_data_;
  Ort::MemoryInfo memory_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::vector<Ort::Value> inputs_;
  Ort::Value output_{nullptr};
  std::vector<const char*> names_;
  Joints action_{};
};
}  // namespace noetix::policy
