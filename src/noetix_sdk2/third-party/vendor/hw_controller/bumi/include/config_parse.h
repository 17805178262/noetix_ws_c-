#pragma once


#include "yaml-cpp/yaml.h"
#include "spdlog_config.h"

namespace YAML {

struct EthercatConfig {
  std::string ifname;
  bool enable_dc;
  int bind_cpu;
  int rt_priority;
  uint64_t cycle_time_ns;
};

template <>
struct convert<EthercatConfig> {
  static bool decode(const Node& node, EthercatConfig& rhs) {
    try {
      rhs.ifname = node["ifname"].as<std::string>();
      rhs.enable_dc = node["enable_dc"].as<bool>();
      rhs.bind_cpu = node["bind_cpu"].as<int>();
      rhs.rt_priority = node["rt_priority"].as<int>();
      rhs.cycle_time_ns = node["cycle_time_ns"].as<uint64_t>();
      return true;
    } catch (const YAML::Exception& e) {
      SPDLOG_ERROR("Parse EtherConfig failed, {}",  e.what());
      return false;
    }
  }
};


struct CanuConfig {
  std::string name;
  bool enable;
  bool imu_enable;
  uint32_t ecat_id;

  struct Actuator {
    std::string name;
    std::string type;
    uint32_t can_id;
  };
  std::vector<Actuator> ch[3];
};

using CanuNetworkConfig = std::vector<CanuConfig>;

template <>
struct convert<CanuNetworkConfig> {
  static bool decode(const Node& node, CanuNetworkConfig& rhs) {
    try {
      for (auto& canu_node : node) {
        CanuConfig canu_cfg;
        canu_cfg.name = canu_node["name"].as<std::string>();
        canu_cfg.ecat_id = canu_node["ecat_id"].as<uint32_t>();
        canu_cfg.enable = canu_node["enable"].as<bool>();
        canu_cfg.imu_enable = canu_node["imu_enable"].as<bool>();
        std::vector<std::string> ch_names{"channel_1", "channel_2", "channel_3"};
        for (size_t i = 0; i < ch_names.size(); i++) {
          if (canu_node[ch_names[i]].IsDefined()) {
            for (auto& actr_node : canu_node[ch_names[i]]) {
              CanuConfig::Actuator actr_cfg;
              actr_cfg.name = actr_node["name"].as<std::string>();
              actr_cfg.type = actr_node["type"].as<std::string>();
              actr_cfg.can_id = actr_node["can_id"].as<uint32_t>();
              canu_cfg.ch[i].push_back(actr_cfg);
            }
          }
        }
        rhs.push_back(canu_cfg);
      }
      return true;
    } catch (const YAML::Exception& e) {
      SPDLOG_ERROR("Parse EtherConfig failed, {}", e.what());
      return false;
    }
  }
};

struct ActuatorConfig {
  std::string name;
  int    direction;
  double  bias;
};

using ActuatorListConfig = std::vector<ActuatorConfig>;


template <>
struct convert<ActuatorListConfig> {
  static bool decode(const Node& node, ActuatorListConfig& rhs) {
    try {
          for (auto& actr_node : node)
          {
            ActuatorConfig actr_cfg;
            actr_cfg.name = actr_node["name"].as<std::string>();
            actr_cfg.direction = actr_node["direction"].as<int>();
            actr_cfg.bias = actr_node["bias"].as<double>();
            rhs.push_back(actr_cfg);
          }
         return true;
    } catch (const YAML::Exception& e) {
      SPDLOG_ERROR("Parse ActuatorConfig failed, {}",  e.what());
      return false;
    }
  }
};

}



