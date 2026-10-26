#ifndef COEFFS_HPP
#define COEFFS_HPP

#include <vector>
#include <cmath>

namespace parallel_ankle {

// 第 n 阶 Chebyshev 多项式 T_n(x) = cos(n · acos(x))。
// x 钳位到 [-1, 1]，域外输入按边界值处理。
inline double chebyshev_T(int n, double x) {
    return std::cos(n * std::acos(std::max(-1.0, std::min(1.0, x))));
}

// T_n(x) 对 x 的解析导数。
// x = ±1 处 acos 公式除零，按端点极限取值：
//   T_n'(+1) = n^2，T_n'(-1) = (-1)^(n+1) · n^2。
inline double chebyshev_T_deriv(int n, double x) {
    if (n == 0) return 0.0;
    double xc = std::max(-1.0, std::min(1.0, x));
    double theta = std::acos(xc);
    double sin_theta = std::sin(theta);
    if (std::fabs(sin_theta) < 1e-12) {
        if (xc >= 0.0) {
            return static_cast<double>(n) * n;
        }
        return (n % 2 == 0 ? -1.0 : 1.0) * static_cast<double>(n) * n;
    }
    return n * std::sin(n * theta) / sin_theta;
}

// 一次性递推预算 T_0..T_order(x) 及其对 x 的导数 dT_0..dT_order(x)。
// 热路径（FK/FK_V_JB 每帧调用）必须用本函数，而不是在双重循环里反复调用
// 上面的标量 chebyshev_T/chebyshev_T_deriv——后者每次都走 acos/sin/cos 超越函数，
// 且同一个 T_j(x) 会被重复计算上百次。
// 递推（纯乘加，无超越函数，|x|<=1 内数值稳定）：
//   T_0=1, T_1=x, T_{k+1} = 2x·T_k - T_{k-1}
//   导数经第二类切比雪夫多项式 U：T_k'(x) = k·U_{k-1}(x)，
//   U_0=1, U_1=2x, U_k = 2x·U_{k-1} - U_{k-2}。
// 端点 x=±1 递推自然给出 T_k'(1)=k^2、T_k'(-1)=(-1)^(k+1)·k^2，
// 与标量版的端点极限一致，无需特殊分支。
// out_T/out_dT 需指向至少 order+1 个 double 的缓冲区（order 为拟合阶数，当前为 10）。
inline void chebyshev_basis(int order, double x, double* out_T, double* out_dT) {
    const double xc = std::max(-1.0, std::min(1.0, x));  // 与标量版同钳位
    out_T[0] = 1.0;
    out_dT[0] = 0.0;
    if (order >= 1) {
        out_T[1] = xc;
        out_dT[1] = 1.0;  // 1·U_0
    }
    double u_prev = 1.0;        // U_0
    double u_curr = 2.0 * xc;   // U_1
    for (int k = 2; k <= order; ++k) {
        out_T[k] = 2.0 * xc * out_T[k - 1] - out_T[k - 2];
        out_dT[k] = static_cast<double>(k) * u_curr;  // k·U_{k-1}
        const double u_next = 2.0 * xc * u_curr - u_prev;  // U_k
        u_prev = u_curr;
        u_curr = u_next;
    }
}

// Chebyshev 拟合配置：阶数、电机角拟合域、任务工作空间域、两张二维系数表。
// 系数表 pit_coeffs/rol_coeffs 按 [i][j] 索引，分别对应 a、b 方向的基函数阶次。
struct ChebyshevConfig {
    int a_order;     // 电机角 a 方向的拟合阶数
    int b_order;     // 电机角 b 方向的拟合阶数
    double a_min;    // 电机角 a 拟合域下界（rad）
    double a_max;    // 电机角 a 拟合域上界（rad）
    double b_min;    // 电机角 b 拟合域下界（rad）
    double b_max;    // 电机角 b 拟合域上界（rad）
    // 任务空间物理工作空间边界（rad）：Pit ±65°、Rol ±20°。
    // 抛光正解用它把"小残差但落在工作空间外其他装配分支"的非物理解判废。
    // 带类内默认值：即使 coeffs.cpp 是旧版或被 Python coeffs.py 重新生成
    // （漏掉这几行），FK_polished 仍能拿到正确的物理边界，不会读到未初始化值。
    // load_coeffs 会以相同数值显式覆盖。
    static constexpr double WS_DEG2RAD = 3.14159265358979323846 / 180.0;
    double pit_min = -65.0 * WS_DEG2RAD;
    double pit_max =  65.0 * WS_DEG2RAD;
    double rol_min = -20.0 * WS_DEG2RAD;
    double rol_max =  20.0 * WS_DEG2RAD;
    std::vector<std::vector<double>> pit_coeffs;  // Pit(a, b) 拟合系数
    std::vector<std::vector<double>> rol_coeffs;  // Rol(a, b) 拟合系数
};

// 填充拟合阶数、拟合域与系数表（系数硬编码，生成自 Python coeffs.py）。
void load_coeffs(ChebyshevConfig& config);

} // namespace parallel_ankle

#endif // COEFFS_HPP
