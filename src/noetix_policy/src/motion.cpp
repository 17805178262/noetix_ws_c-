// 中文说明：在C++运行时直接读取现有50fps NPZ动作资源，不通过Python转换或执行脚本。
// 使用minizip解压并验证CRC，解析NumPy标准NPY头，检查dtype、形状、大小、有限值和关节名称。
// 支持小端C/Fortran顺序float32/float64与ASCII关节名（NumPy Unicode或bytes存储）；不允许pickle。
// 读取仅发生在策略初始化，周期内按帧索引取值并执行与Python相同的float32航向旋转。
#include <minizip/unzip.h>

#include <cstring>
#include <map>
#include <numeric>
#include <regex>

#include "noetix_policy_cpp/core.hpp"
namespace noetix::policy {
namespace {
struct Array {
  std::string dtype;
  std::vector<size_t> shape;
  std::vector<unsigned char> bytes;
  size_t offset = 0;
  bool fortran = false;
};
Array read_array(unzFile zip, const char* name) {
  require(unzLocateFile(zip, name, 1) == UNZ_OK, std::string("NPZ missing ") + name);
  unz_file_info64 info{};
  require(unzGetCurrentFileInfo64(zip, &info, nullptr, 0, nullptr, 0, nullptr, 0) == UNZ_OK,
          "Invalid NPZ entry");
  require(info.uncompressed_size >= 10 && info.uncompressed_size <= 256 * 1024 * 1024,
          "Invalid NPZ entry size");
  require(unzOpenCurrentFile(zip) == UNZ_OK, "Cannot open NPZ entry");
  Array out;
  out.bytes.resize(info.uncompressed_size);
  size_t count = 0;
  while (count < out.bytes.size()) {
    int n = unzReadCurrentFile(zip, out.bytes.data() + count,
                               std::min<size_t>(65536, out.bytes.size() - count));
    if (n <= 0) {
      unzCloseCurrentFile(zip);
      throw std::runtime_error("Truncated NPZ");
    }
    count += n;
  }
  require(unzCloseCurrentFile(zip) == UNZ_OK, "NPZ CRC mismatch");
  auto& b = out.bytes;
  require(std::memcmp(b.data(), "\x93NUMPY", 6) == 0, "Invalid NPY magic");
  size_t base = b[6] == 1 ? 10 : 12;
  require((b[6] >= 1 && b[6] <= 3) && b.size() >= base, "Unsupported NPY version");
  size_t length = b[8] + (size_t(b[9]) << 8);
  if (base == 12)
    length += (size_t(b[10]) << 16) + (size_t(b[11]) << 24);
  require(length <= b.size() - base, "Invalid NPY header length");
  out.offset = base + length;
  std::string header(reinterpret_cast<const char*>(b.data() + base), length);
  std::smatch match;
  require(std::regex_search(header, match, std::regex("['\"]descr['\"]\\s*:\\s*['\"]([^'\"]+)")),
          "NPY dtype missing");
  out.dtype = match[1];
  out.fortran = std::regex_search(header, std::regex("['\"]fortran_order['\"]\\s*:\\s*True"));
  require(std::regex_search(header, match, std::regex("['\"]shape['\"]\\s*:\\s*\\(([^)]*)\\)")),
          "NPY shape missing");
  std::string shape = match[1];
  std::regex number("[0-9]+");
  for (std::sregex_iterator i(shape.begin(), shape.end(), number), end; i != end; ++i)
    out.shape.push_back(std::stoull(i->str()));
  size_t elements = 1;
  for (auto size : out.shape) {
    require(size > 0 && elements <= 256 * 1024 * 1024 / size, "Invalid NPY shape");
    elements *= size;
  }
  require(out.dtype.size() >= 3 && out.dtype[0] != '>', "Unsupported NPY endianness");
  size_t item = std::stoul(out.dtype.substr(2));
  if (out.dtype[1] == 'U')
    item *= 4;
  require(
      item > 0 && elements <= 256 * 1024 * 1024 / item && elements * item == b.size() - out.offset,
      "NPY data size mismatch");
  return out;
}
double numeric(const Array& a, size_t index) {
  if (a.fortran && a.shape.size() > 1) {
    std::vector<size_t> coordinates(a.shape.size());
    for (size_t i = a.shape.size(); i-- > 0;) {
      coordinates[i] = index % a.shape[i];
      index /= a.shape[i];
    }
    size_t stride = 1;
    index = 0;
    for (size_t i = 0; i < a.shape.size(); ++i) {
      index += coordinates[i] * stride;
      stride *= a.shape[i];
    }
  }
  if (a.dtype == "<f4" || a.dtype == "=f4") {
    float v;
    std::memcpy(&v, a.bytes.data() + a.offset + index * 4, 4);
    require(std::isfinite(v), "Motion contains NaN/Inf");
    return v;
  }
  if (a.dtype == "<f8" || a.dtype == "=f8") {
    double v;
    std::memcpy(&v, a.bytes.data() + a.offset + index * 8, 8);
    require(std::isfinite(v), "Motion contains NaN/Inf");
    return v;
  }
  throw std::invalid_argument("Motion numeric dtype must be float32/float64");
}
F4 motion_yaw(F4 q) {
  q = normalize(q);
  F3 x = rotate(q, F3(1, 0, 0)), z = rotate(q, F3(0, 0, 1));
  F3 h = x.head<2>().norm() > .1f ? x : (x[2] < 0 ? z : F3(-z));
  double half = double(std::atan2(h[1], h[0])) * .5;
  return {float(std::cos(half)), 0, 0, float(std::sin(half))};
}
}  // namespace
Motion::Motion(const std::string& path, const std::vector<std::string>& joints) {
  unzFile zip = unzOpen64(path.c_str());
  require(zip != nullptr, "Cannot open motion NPZ: " + path);
  try {
    auto fps = read_array(zip, "fps.npy"), names = read_array(zip, "joint_names.npy"),
         q = read_array(zip, "joint_pos.npy"), p = read_array(zip, "root_pos.npy"),
         r = read_array(zip, "root_quat.npy");
    require(fps.shape.empty() && std::abs(numeric(fps, 0) - 50.) <= .0005, "Motion must be 50fps");
    require(names.shape.size() == 1 && (names.dtype[1] == 'U' || names.dtype[1] == 'S'),
            "Invalid motion joint names");
    size_t chars = std::stoul(names.dtype.substr(2)), width = names.dtype[1] == 'U' ? 4 : 1;
    std::vector<std::string> order;
    for (size_t i = 0; i < names.shape[0]; ++i) {
      std::string name;
      for (size_t j = 0; j < chars; ++j) {
        unsigned code = 0;
        std::memcpy(&code, names.bytes.data() + names.offset + (i * chars + j) * width, width);
        if (code == 0)
          break;
        require(code < 128, "Joint names must be ASCII");
        name += char(code);
      }
      order.push_back(name);
    }
    require(q.shape.size() == 2 && q.shape[1] == order.size() &&
                p.shape == std::vector<size_t>({q.shape[0], 3}) &&
                r.shape == std::vector<size_t>({q.shape[0], 4}),
            "Invalid motion shapes");
    std::vector<size_t> indices;
    for (auto& name : joints) {
      auto it = std::find(order.begin(), order.end(), name);
      require(it != order.end(), "Missing motion joint: " + name);
      indices.push_back(it - order.begin());
    }
    poses_.resize(q.shape[0]);
    for (size_t i = 0; i < poses_.size(); ++i) {
      auto& pose = poses_[i];
      for (int j = 0; j < 3; ++j)
        pose.position[j] = float(numeric(p, i * 3 + j));
      F4 quat;
      for (int j = 0; j < 4; ++j)
        quat[j] = float(numeric(r, i * 4 + j));
      pose.quaternion = normalize(quat).cast<double>();
      for (size_t j = 0; j < indices.size(); ++j)
        pose.joints[j] = float(numeric(q, i * q.shape[1] + indices[j]));
    }
    unzClose(zip);
  } catch (...) {
    unzClose(zip);
    throw;
  }
}
void Motion::align(const F4& q) {
  if (!heading_)
    heading_ = multiply(motion_yaw(q), conjugate(motion_yaw(poses_[0].quaternion.cast<float>())));
}
Pose Motion::frame(int index) const {
  require(heading_.has_value(), "Motion heading not aligned");
  index = std::clamp(index, 0, int(poses_.size()) - 1);
  auto p = poses_[index];
  F3 origin = poses_[0].position.cast<float>();
  origin[2] = 0;
  p.position = F3(origin + rotate(normalize(*heading_), F3(p.position.cast<float>() - origin)))
                   .cast<double>();
  p.quaternion = multiply(*heading_, F4(p.quaternion.cast<float>())).cast<double>();
  return p;
}
}  // namespace noetix::policy
