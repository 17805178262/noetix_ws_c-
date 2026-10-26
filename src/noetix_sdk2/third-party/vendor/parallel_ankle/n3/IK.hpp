#ifndef IK_HPP
#define IK_HPP

namespace parallel_ankle {

/**
 * @brief 逆运动学求解
 * @param Pit 俯仰角（弧度）
 * @param Rol 横滚角（弧度）
 * @param a 输出：关节角a（弧度）
 * @param b 输出：关节角b（弧度）
 */
void IK(double Pit, double Rol, double& a, double& b);

} // namespace parallel_ankle

#endif // IK_HPP
