// 中文说明：实现模型契约校验、CPU线程选择、CUDA显式选择及内存复用推理。
// ONNX Runtime 只认识 Spec 和稳定的 float 缓冲区，不持有任何策略算法对象。
// 会话缓存只保存模型，策略的本体/动作/参考历史相互独立。模型SHA256在加载时计算并记录。
#include "noetix_policy_cpp/inference.hpp"

#include <openssl/evp.h>
#include <sched.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
namespace noetix::policy {
namespace {
Ort::Env& environment() {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "noetix_policy_cpp");
  return env;
}
std::string env_or(const char* key, const char* fallback) {
  const char* value = std::getenv(key);
  return value ? value : fallback;
}
std::string digest(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  require(bool(file), "Cannot read ONNX model: " + path);
  auto* context = EVP_MD_CTX_new();
  require(context != nullptr, "SHA allocation failed");
  EVP_DigestInit_ex(context, EVP_sha256(), nullptr);
  std::array<char, 65536> buffer{};
  while (file) {
    file.read(buffer.data(), buffer.size());
    EVP_DigestUpdate(context, buffer.data(), file.gcount());
  }
  std::array<unsigned char, 32> hash{};
  unsigned size = 0;
  EVP_DigestFinal_ex(context, hash.data(), &size);
  EVP_MD_CTX_free(context);
  std::ostringstream out;
  for (auto byte : hash)
    out << std::hex << std::setw(2) << std::setfill('0') << int(byte);
  return out.str();
}
double percentile(std::vector<double> v, double p) {
  std::sort(v.begin(), v.end());
  double index = (v.size() - 1) * p;
  size_t i = size_t(index);
  return v[i] + (v[std::min(i + 1, v.size() - 1)] - v[i]) * (index - i);
}
Ort::Session create(const std::string& path, int threads, bool cuda) {
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(threads);
  options.SetInterOpNumThreads(1);
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  options.AddConfigEntry("session.intra_op.allow_spinning", "0");
  options.AddConfigEntry("session.inter_op.allow_spinning", "0");
  if (cuda) {
    auto providers = Ort::GetAvailableProviders();
    require(
        std::find(providers.begin(), providers.end(), "CUDAExecutionProvider") != providers.end(),
        "CUDA requested but this ORT has no CUDA EP");
    OrtCUDAProviderOptionsV2* settings = nullptr;
    auto& api = Ort::GetApi();
    Ort::ThrowOnError(api.CreateCUDAProviderOptions(&settings));
    try {
      const char* keys[] = {"device_id", "use_tf32"};
      const char* values[] = {"0", "0"};
      Ort::ThrowOnError(api.UpdateCUDAProviderOptions(settings, keys, values, 2));
      options.AppendExecutionProvider_CUDA_V2(*settings);
      api.ReleaseCUDAProviderOptions(settings);
    } catch (...) {
      api.ReleaseCUDAProviderOptions(settings);
      throw;
    }
  }
  return Ort::Session(environment(), path.c_str(), options);
}
void validate(Ort::Session& session, const Spec& spec, std::vector<int64_t>& output_shape) {
  require(session.GetInputCount() == spec.inputs.size(), "ONNX input count mismatch");
  Ort::AllocatorWithDefaultOptions alloc;
  std::map<std::string, size_t> actual;
  for (size_t i = 0; i < session.GetInputCount(); ++i)
    actual.emplace(session.GetInputNameAllocated(i, alloc).get(), i);
  for (const auto& input : spec.inputs) {
    require(actual.count(input.name), "Missing ONNX input " + input.name);
    auto info = session.GetInputTypeInfo(actual.at(input.name));
    auto tensor = info.GetTensorTypeAndShapeInfo();
    auto shape = tensor.GetShape();
    require(tensor.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
                shape.size() == input.shape.size(),
            "Wrong ONNX input dtype/rank");
    for (size_t j = 0; j < shape.size(); ++j)
      require(shape[j] < 0 || shape[j] == input.shape[j], "Wrong ONNX input shape");
  }
  bool found = false;
  for (size_t i = 0; i < session.GetOutputCount(); ++i) {
    auto name = session.GetOutputNameAllocated(i, alloc);
    if (spec.output == name.get()) {
      auto info = session.GetOutputTypeInfo(i);
      auto tensor = info.GetTensorTypeAndShapeInfo();
      output_shape = tensor.GetShape();
      require(tensor.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
                  !output_shape.empty() && output_shape.back() == spec.dof(),
              "Wrong ONNX output");
      size_t count = 1;
      for (auto& dim : output_shape) {
        if (dim < 0)
          dim = 1;
        count *= dim;
      }
      require(count == size_t(spec.dof()), "Invalid action tensor size");
      found = true;
    }
  }
  require(found, "Missing ONNX output " + spec.output);
}
struct Measurement {
  double median = 0, p95 = 0;
  Joints action{};
};
Measurement measure(Ort::Session& session, const Spec& spec, const std::vector<int64_t>& shape) {
  std::vector<std::vector<float>> values;
  std::vector<Ort::Value> tensors;
  std::vector<const char*> names;
  auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  for (const auto& input : spec.inputs) {
    size_t size = 1;
    for (auto d : input.shape)
      size *= d;
    values.emplace_back(size, 0.f);
  }
  if (spec.kind == Kind::gmt) {
    auto& cmd = values[0];
    for (int f = 0; f < 8; ++f) {
      cmd[f * 3 + 2] = .7245f;
      cmd[24 + f * 6] = 1;
      cmd[24 + f * 6 + 4] = 1;
      std::copy(spec.default_q.begin(), spec.default_q.end(), cmd.begin() + 72 + f * spec.dof());
    }
    auto& policy = values[1];
    for (int f = 0; f < 7; ++f) {
      policy[21 + f * 3 + 2] = -1;
      std::copy(spec.default_q.begin(), spec.default_q.end(), policy.begin() + 42 + f * spec.dof());
    }
  }
  for (size_t i = 0; i < values.size(); ++i) {
    auto& value = values[i];
    const auto& input = spec.inputs[i];
    tensors.push_back(Ort::Value::CreateTensor<float>(memory, value.data(), value.size(),
                                                      input.shape.data(), input.shape.size()));
    names.push_back(input.name.c_str());
  }
  Measurement m;
  auto output = Ort::Value::CreateTensor<float>(memory, m.action.data(), spec.dof(), shape.data(),
                                                shape.size());
  const char* output_name = spec.output.c_str();
  std::vector<double> times;
  for (int i = 0; i < 40; ++i) {
    auto start = std::chrono::steady_clock::now();
    session.Run(Ort::RunOptions{nullptr}, names.data(), tensors.data(), tensors.size(),
                &output_name, &output, 1);
    auto end = std::chrono::steady_clock::now();
    for (int j = 0; j < spec.dof(); ++j)
      require(std::isfinite(m.action[j]), "Warmup contains NaN/Inf");
    if (i >= 8) {
      times.push_back(std::chrono::duration<double, std::milli>(end - start).count());
      std::this_thread::sleep_until(start + std::chrono::milliseconds(20));
    }
  }
  m.median = percentile(times, .5);
  m.p95 = percentile(times, .95);
  return m;
}
}  // namespace
Model::Model(const Spec& spec, const std::string& path) {
  sha256 = digest(path);
  backend = spec.kind == Kind::gmt ? env_or("NOETIX_GMT_BACKEND", "cpu") : "cpu";
  require(backend == "cpu" || backend == "cuda", "NOETIX_GMT_BACKEND must be cpu or cuda");
  std::string setting = spec.kind == Kind::gmt ? env_or("NOETIX_GMT_ORT_THREADS", "2") : "1";
  require(setting == "auto" || setting == "1" || setting == "2" || setting == "4",
          "NOETIX_GMT_ORT_THREADS must be auto/1/2/4");
  if (backend == "cuda") {
    threads = 1;
    session = create(path, 1, true);
    validate(session, spec, output_shape);
    measure(session, spec, output_shape);
    report = "CUDA FP32; TF32 off";
    return;
  }
  std::vector<int> candidates{setting == "auto" ? 1 : std::stoi(setting)};
  cpu_set_t allowed;
  CPU_ZERO(&allowed);
  if (setting == "auto" && sched_getaffinity(0, sizeof(allowed), &allowed) == 0 &&
      CPU_COUNT(&allowed) >= 2)
    candidates.push_back(2);
  Measurement best;
  bool selected = false;
  std::ostringstream result;
  result << "configured_threads=" << setting;
  for (int count : candidates) {
    auto current = create(path, count, false);
    std::vector<int64_t> shape;
    validate(current, spec, shape);
    if (spec.kind != Kind::gmt) {
      session = std::move(current);
      threads = count;
      output_shape = shape;
      break;
    }
    auto timing = measure(current, spec, shape);
    result << "; threads=" << count << ": p50=" << timing.median << "/p95=" << timing.p95 << " ms";
    bool same = true;
    if (selected)
      for (int j = 0; j < spec.dof(); ++j)
        if (std::abs(timing.action[j] - best.action[j]) > 1e-5f + 1e-5f * std::abs(best.action[j]))
          same = false;
    if (!same) {
      result << " rejected(output mismatch)";
      continue;
    }
    if (!selected || (timing.median < best.median * .95 && timing.p95 < best.p95 * .90)) {
      session = std::move(current);
      threads = count;
      output_shape = shape;
      best = timing;
      selected = true;
    }
  }
  report = result.str();
}
std::shared_ptr<Model> load_model(const Spec& spec, const std::string& path) {
  // Env先于缓存构造、晚于缓存析构，确保所有Session释放后才销毁ORT环境。
  (void)environment();
  static std::mutex mutex;
  static std::map<std::string, std::shared_ptr<Model>> cache;
  std::lock_guard<std::mutex> lock(mutex);
  std::string key = path + ":" + env_or("NOETIX_GMT_BACKEND", "cpu") + ":" +
                    env_or("NOETIX_GMT_ORT_THREADS", "2");
  auto it = cache.find(key);
  if (it != cache.end())
    return it->second;
  auto value = std::make_shared<Model>(spec, path);
  cache.emplace(key, value);
  return value;
}
Inference::Inference(const Spec& config, std::vector<std::vector<float>>& data,
                     std::shared_ptr<Model> value)
    : model(std::move(value)), config_(config), input_data_(data) {
  require(config.inputs.size() == input_data_.size(), "Input binding count mismatch");
  inputs_.reserve(config.inputs.size());
  names_.reserve(config.inputs.size());
  for (size_t i = 0; i < config.inputs.size(); ++i) {
    const auto& input = config.inputs[i];
    auto& values = input_data_[i];
    inputs_.push_back(Ort::Value::CreateTensor<float>(memory_, values.data(), values.size(),
                                                     input.shape.data(), input.shape.size()));
    names_.push_back(input.name.c_str());
  }
  output_ = Ort::Value::CreateTensor<float>(memory_, action_.data(), config.dof(),
                                           model->output_shape.data(), model->output_shape.size());
}
const float* Inference::run() {
  for (const auto& values : input_data_)
    for (float x : values)
      require(std::isfinite(x), "Observation contains NaN/Inf");
  const char* name = config_.output.c_str();
  model->session.Run(Ort::RunOptions{nullptr}, names_.data(), inputs_.data(), inputs_.size(), &name,
                     &output_, 1);
  for (int j = 0; j < config_.dof(); ++j)
    require(std::isfinite(action_[j]), "ONNX action contains NaN/Inf");
  return action_.data();
}
}  // namespace noetix::policy
