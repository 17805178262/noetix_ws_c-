#pragma once
#include <nlohmann/json.hpp>
#include <array>
#include <optional>
#include <sstream>
#include <cmath>
#include <stdexcept>

namespace gmr {
using PicoJoints = std::array<std::array<double, 7>, 24>;
inline nlohmann::json decodePicoObject(const nlohmann::json& value) {
    return value.is_string() ? nlohmann::json::parse(value.get<std::string>()) : value;
}
// Some service versions wrap value/Body in JSON strings; others use objects.
inline std::optional<PicoJoints> parsePicoBody(const char* text) {
    if (!text) throw std::invalid_argument("Pico JSON is null");
    const auto root = nlohmann::json::parse(text);
    if (!root.contains("value")) return std::nullopt;
    const auto value = decodePicoObject(root.at("value"));
    if (!value.contains("Body")) return std::nullopt;
    const auto body = decodePicoObject(value.at("Body"));
    const auto& joints = body.at("joints");
    if (!joints.is_array() || joints.size() < 24)
        throw std::runtime_error("Body.joints must contain at least 24 poses");
    PicoJoints result{};
    for (size_t i = 0; i < result.size(); ++i) {
        const auto& pose = joints.at(i).at("p");
        if (pose.is_string()) {
            std::istringstream stream(pose.get<std::string>());
            std::string token;
            for (double& component : result[i]) {
                if (!std::getline(stream, token, ','))
                    throw std::runtime_error("Pico pose needs seven components");
                size_t used = 0;
                component = std::stod(token, &used);
                if (token.find_first_not_of(" \t\r\n", used) != std::string::npos)
                    throw std::runtime_error("Invalid Pico pose component");
            }
            if (std::getline(stream, token, ','))
                throw std::runtime_error("Pico pose has extra components");
        } else if (pose.is_array() && pose.size() == 7) {
            for (size_t j = 0; j < 7; ++j) result[i][j] = pose.at(j).get<double>();
        } else throw std::runtime_error("Unsupported Pico pose representation");
        double norm = 0;
        for (size_t j = 0; j < 7; ++j) {
            if (!std::isfinite(result[i][j])) throw std::runtime_error("Non-finite Pico pose");
            if (j >= 3) norm += result[i][j] * result[i][j];
        }
        if (!std::isfinite(norm) || norm < 1e-16)
            throw std::runtime_error("Invalid Pico quaternion");
    }
    return result;
}
} // namespace gmr
