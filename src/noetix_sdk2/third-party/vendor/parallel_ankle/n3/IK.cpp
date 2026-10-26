#include "IK.hpp"
#include <cmath>
#include <iostream>


namespace parallel_ankle {

// 机构几何参数 (同步自 LV_Z_6_12/main/IK.py)
constexpr double U = 43;
constexpr double V = 26.25;
constexpr double W = 0;

constexpr double UA = 18;        // -X_FALAN
constexpr double VA = 26.25;      // Y_FALAN

constexpr double L_LONG = 230;
constexpr double L_SHORT = 150;
constexpr double H_LONG = 230;   // Z_LONG
constexpr double H_SHORT = 150;  // Z_SHORT
constexpr double R = 55;

constexpr double MOTOR_PI_OFFSET_hi = 0.001423193978;
constexpr double MOTOR_PI_OFFSET_low = 0.002182788710;
void IK(double Pit, double Rol, double& a, double& b) {
    double cos_Pit = std::cos(Pit);
    double sin_Pit = std::sin(Pit);
    double cos_Rol = std::cos(Rol);
    double sin_Rol = std::sin(Rol);

    // 计算左右末端点位置
    double TEMP_L = -V * sin_Rol - W * cos_Rol;
    double TEMP_R = V * sin_Rol - W * cos_Rol;

    double X_left = U * cos_Pit + sin_Pit * TEMP_L;
    double Y_left = -V * cos_Rol + W * sin_Rol;
    double Z_left = -U * sin_Pit + cos_Pit * TEMP_L;

    double X_right = U * cos_Pit + sin_Pit * TEMP_R;
    double Y_right = V * cos_Rol + W * sin_Rol;
    double Z_right = -U * sin_Pit + cos_Pit * TEMP_R;

    // 构建向量
    double LP[3] = {X_left + UA, Y_left + VA, Z_left - H_LONG};
    double RP[3] = {X_right + UA, Y_right - VA, Z_right - H_SHORT};

    // 计算长度
    double LLP = std::sqrt(LP[0] * LP[0] + LP[2] * LP[2]);
    double RRP = std::sqrt(RP[0] * RP[0] + RP[2] * RP[2]);

    // 计算中间变量
    double LT = (LP[0] * LP[0] + LP[1] * LP[1] + LP[2] * LP[2] + R * R - L_LONG * L_LONG) / (2 * R);
    double RT = (RP[0] * RP[0] + RP[1] * RP[1] + RP[2] * RP[2] + R * R - L_SHORT * L_SHORT) / (2 * R);

    // 计算关节角
    double arg_a = LT / LLP;
    double arg_b = RT / RRP;

    // 限制参数在有效范围内
    arg_a = std::max(-1.0, std::min(1.0, arg_a));
    arg_b = std::max(-1.0, std::min(1.0, arg_b));

    a = -std::acos(arg_a) - std::atan2(LP[2], LP[0]) - MOTOR_PI_OFFSET_hi;
    b = -std::acos(arg_b) - std::atan2(RP[2], RP[0]) - MOTOR_PI_OFFSET_low;
}

} // namespace parallel_ankle
