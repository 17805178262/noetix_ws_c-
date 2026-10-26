#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include "coeffs.hpp"

namespace parallel_ankle {

/**
 * @brief 计算速度雅可比矩阵 d(Pit,Rol)/d(a,b)
 * @param a 关节角a（弧度）
 * @param b 关节角b（弧度）
 * @param Pit 当前俯仰角（弧度）
 * @param Rol 当前横滚角（弧度）
 * @param jacob 输出：4元素数组 [J11, J12, J21, J22]
 * @param config Chebyshev拟合配置（包含系数矩阵）
 */
void FK_V_JB(double a, double b, double Pit, double Rol, double jacob[4], const ChebyshevConfig& config);

} // namespace parallel_ankle

#endif // JACOBIAN_HPP
