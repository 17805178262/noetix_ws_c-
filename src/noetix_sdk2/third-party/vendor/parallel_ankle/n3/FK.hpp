#ifndef FK_HPP
#define FK_HPP

#include "coeffs.hpp"

namespace parallel_ankle {

/**
 * @brief 正运动学求解（基于Chebyshev多项式拟合）—— 种子解
 * @param a 关节角a（弧度）
 * @param b 关节角b（弧度）
 * @param Pit 输出：俯仰角（弧度）
 * @param Rol 输出：横滚角（弧度）
 * @param config Chebyshev拟合配置（包含系数矩阵）
 *
 * 这是纯 Chebyshev 多项式求值，无迭代、无超越函数，给出任务角的"种子"。
 * 精度受拟合阶数限制（物理点往返残差约 1e-4 rad），需要高精度时用 FK_polished。
 */
void FK(double a, double b, double& Pit, double& Rol, const ChebyshevConfig& config);

/**
 * @brief 抛光正解的结果包。
 *
 * 一次 FK_polished 调用同时给出下游需要的全部量，避免适配层再分别调用
 * FK / IK / FK_V_JB：
 * - Pit/Rol：抛光后的任务角；
 * - J：工作点正雅可比 J_fk = d(Pit,Rol)/d(a,b)，行主序 2x2
 *      [dPit/da, dPit/db; dRol/da, dRol/db]，供速度/力矩映射与缓存；
 * - res_a/res_b：抛光末轮用精确 IK 测得的电机空间往返残差（rad），
 *      直接用作 FK->IK 往返一致性有效性判据；
 * - in_domain：抛光解是否落在物理工作空间（Pit ±65°、Rol ±20°，含少量裕量）内。
 *      拟合盒内有闭链不可达的电机角组合，其 Chebyshev 种子是外推垃圾值；若放任
 *      Newton 迭代，残差可能被压到"工作空间外的其他装配分支"上而误判为有效。
 *      域外种子直接拒绝抛光、in_domain=false，由上层判废；
 * - iters：实际执行的 Newton 抛光步数（域外种子为 0）。
 */
struct FKResult {
    double Pit = 0.0;
    double Rol = 0.0;
    double J[4] = {0.0, 0.0, 0.0, 0.0};
    double res_a = 0.0;
    double res_b = 0.0;
    int    iters = 0;
    bool   in_domain = false;
};

/**
 * @brief 正运动学求解（Chebyshev 种子 + 1~2 次 Newton 抛光）
 * @param a 实测关节角a（弧度）
 * @param b 实测关节角b（弧度）
 * @param out 输出：抛光结果（任务角、雅可比、往返残差、迭代次数）
 * @param config Chebyshev拟合配置
 * @param newton_iters Newton 抛光步数上限（默认 2）
 * @param tol 电机空间往返残差收敛阈值（rad），残差低于此值即提前停止
 *
 * 思路：
 * 1. Chebyshev 拟合 FK 显式给出任务角种子 x0（多项式求值，无迭代）；
 * 2. 以精确解析 IK 为真值模型，在电机空间定义残差 e(x) = IK(x) - q_meas；
 * 3. 用 Chebyshev 解析雅可比 J_fk = d(Pit,Rol)/d(a,b) 作 Newton 矩阵，
 *    任务空间修正 dx = -J_fk·e（dq = J_ik·dx，J_fk = J_ik^{-1}，令 dq = -e）。
 *
 * J_fk 在实测电机角 q_meas 处一次性求值并冻结：它既是 Newton 矩阵，又是
 * 速度/力矩映射要用的工作点雅可比；抛光迭代点与 q_meas 只差种子残差（~1e-4 rad），
 * 换点重估对收敛无实质收益，反而多一次最贵的雅可比求值。
 *
 * 种子残差约 1e-4 rad，Newton 每步把残差压低约一个"雅可比相对误差"量级，
 * 1~2 步即可把往返残差从 1e-4 压到 1e-8~1e-11 rad（解析 IK 模型精度极限），
 * 计算量仅比纯 Chebyshev 正解多 1~3 次廉价的解析 IK 求值。
 */
void FK_polished(double a, double b, FKResult& out, const ChebyshevConfig& config,
                 int newton_iters = 2, double tol = 1e-11);

} // namespace parallel_ankle

#endif // FK_HPP
