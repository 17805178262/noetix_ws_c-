#include "FK.hpp"
#include "IK.hpp"
#include "Jacobian.hpp"
#include "coeffs.hpp"
#include <cmath>

namespace parallel_ankle {

// 正运动学：电机角 (a, b) -> 任务角 (Pit, Rol)。
// 电机角先线性归一化到 Chebyshev 定义域 [-1, 1]，
// 再用二维 Chebyshev 级数求和：Pit/Rol = ΣΣ c[i][j] · T_i(a_norm) · T_j(b_norm)。
// 注意：拟合只在 config 的拟合域内有效，域外为多项式外推，不应使用。
void FK(double a, double b, double& Pit, double& Rol, const ChebyshevConfig& config) {
    double a_norm = 2.0 * (a - config.a_min) / (config.a_max - config.a_min) - 1.0;
    double b_norm = 2.0 * (b - config.b_min) / (config.b_max - config.b_min) - 1.0;

    // 递推预算两个方向的基函数（各 order+1 个值，纯乘加），
    // 避免在内层双重循环里重复调用 acos/cos 版 chebyshev_T。
    // 缓冲区 32：当前拟合阶数为 10，留足余量。
    double Ta[32], Tb[32], dTmp[32];
    chebyshev_basis(config.a_order, a_norm, Ta, dTmp);
    chebyshev_basis(config.b_order, b_norm, Tb, dTmp);

    Pit = 0.0;
    Rol = 0.0;
    for (int i = 0; i <= config.a_order; ++i) {
        const double tai = Ta[i];
        const double* cp = config.pit_coeffs[i].data();
        const double* cr = config.rol_coeffs[i].data();
        for (int j = 0; j <= config.b_order; ++j) {
            const double tbj = Tb[j];
            Pit += cp[j] * tai * tbj;
            Rol += cr[j] * tai * tbj;
        }
    }
}

// 正运动学（Chebyshev 种子 + Newton 抛光）：
// 1) Chebyshev FK 给出任务角种子（纯多项式，无迭代）；
// 2) 精确解析 IK 测电机空间残差 e = IK(x) - q_meas；
// 3) Chebyshev 解析雅可比 J_fk 作 Newton 矩阵，任务空间修正 dx = -J_fk·e。
// 残差定义在电机空间、修正量在任务空间：由 dq = J_ik·dx 与 J_fk = J_ik^{-1}，
// 令 dq = -e 即得 dx = -J_fk·e。
//
// J_fk 在实测电机角 q_meas 处只求一次并冻结：它同时是 Newton 矩阵和速度/力矩
// 映射要用的工作点雅可比；抛光迭代点与 q_meas 只差种子残差（~1e-4 rad），
// 换点重估改变的只是 J 在 1e-4 邻域内的取值变化，远小于 J 本身对真值的拟合误差，
// 对收敛没有实质帮助，反而多一次最贵的雅可比求值。
//
// 工作空间域门控：拟合盒内存在闭链不可达的电机角组合，其 Chebyshev 种子是外推
// 垃圾值（任务角可达上百弧度）。从这种种子放任 Newton 迭代，会把残差压到
// "工作空间外的其他装配分支"（IK 含 acos/atan2，非工作空间位姿也能反算出电机角），
// 得到小残差但非物理的假根。因此种子落在物理工作空间（Pit ±65°、Rol ±20°，
// 加少量裕量接纳边界点拟合误差）之外时，直接拒绝抛光、in_domain=false。
// 物理上合法的反馈种子必在工作空间内；工作空间内 IK(x)=q 本就意味着 q 可达。
//
// 循环在每轮顶部用精确 IK 测残差，足够小即提前退出，否则做一次 dx = -J_fk·e；
// 退出时 res_a/res_b 始终对应当前任务角的新鲜往返残差，可直接作有效性判据。
void FK_polished(double a, double b, FKResult& out, const ChebyshevConfig& config,
                 int newton_iters, double tol) {
    if (newton_iters < 0) newton_iters = 0;

    // --- Chebyshev 种子 ---
    FK(a, b, out.Pit, out.Rol, config);

    // --- 工作点正雅可比（Newton 矩阵，同时供速度/力矩映射与缓存复用）---
    FK_V_JB(a, b, out.Pit, out.Rol, out.J, config);

    // 物理工作空间域判定（带裕量，接纳边界点种子的 ~1e-5 量级拟合误差）。
    // 实测种子最大拟合误差约 3.3e-5 rad，裕量取 0.02 rad（≈1.15°，约为拟合误差
    // 的 600 倍）既不会误剔边界点，又把可接受位姿的外扩限制在 ~1° 以内。
    const double kMargin = 0.02;  // rad ≈ 1.15°
    const double pit_lim = config.pit_max + kMargin;
    const double rol_lim = config.rol_max + kMargin;
    const bool seed_in_domain =
        std::fabs(out.Pit) <= pit_lim && std::fabs(out.Rol) <= rol_lim;

    double ea = 0.0;
    double eb = 0.0;

    if (!seed_in_domain) {
        // 外推垃圾种子：不抛光，仅测一次往返残差（必然很大），交上层判废。
        double ak = 0.0;
        double bk = 0.0;
        IK(out.Pit, out.Rol, ak, bk);
        ea = ak - a;
        eb = bk - b;
        out.iters = 0;
        out.in_domain = false;
        out.res_a = ea;
        out.res_b = eb;
        return;
    }

    const double j0 = out.J[0];
    const double j1 = out.J[1];
    const double j2 = out.J[2];
    const double j3 = out.J[3];

    // --- Newton 抛光 ---
    int it = 0;
    for (; it <= newton_iters; ++it) {
        double ak = 0.0;
        double bk = 0.0;
        IK(out.Pit, out.Rol, ak, bk);  // 精确解析模型残差
        ea = ak - a;
        eb = bk - b;

        if (std::fabs(ea) < tol && std::fabs(eb) < tol) {
            break;  // 已收敛
        }
        if (it == newton_iters) {
            break;  // 用完抛光步数，保留当前（最后一轮）残差
        }

        // 任务空间 Newton 修正：dx = -J_fk · e
        out.Pit -= j0 * ea + j1 * eb;
        out.Rol -= j2 * ea + j3 * eb;
    }

    // 抛光解仍须落在物理工作空间内（合法点从域内种子出发只会在域内收敛）。
    out.in_domain =
        std::fabs(out.Pit) <= pit_lim && std::fabs(out.Rol) <= rol_lim;
    out.iters = it;
    out.res_a = ea;
    out.res_b = eb;
}

} // namespace parallel_ankle
