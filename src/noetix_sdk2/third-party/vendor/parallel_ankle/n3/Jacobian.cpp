#include "Jacobian.hpp"
#include "coeffs.hpp"
#include <cmath>

namespace parallel_ankle {

// 正速度雅可比 J_fk = d(Pit, Rol) / d(a, b)，由 Chebyshev 拟合式解析求导得到。
// 输出布局（行主序 2x2）：
//   jacob[0] = dPit/da, jacob[1] = dPit/db
//   jacob[2] = dRol/da, jacob[3] = dRol/db
// Pit/Rol 入参为保持接口对称保留，内部不使用。
void FK_V_JB(double a, double b, double /*Pit*/, double /*Rol*/, double jacob[4], const ChebyshevConfig& config) {
    // 电机角归一化值及其对原变量的尺度因子（链导：d/da = da_norm · d/da_norm）。
    double a_norm = 2.0 * (a - config.a_min) / (config.a_max - config.a_min) - 1.0;
    double b_norm = 2.0 * (b - config.b_min) / (config.b_max - config.b_min) - 1.0;

    const double sa = 2.0 / (config.a_max - config.a_min);
    const double sb = 2.0 / (config.b_max - config.b_min);

    // 递推预算两个方向的基函数及其导数（纯乘加，无 acos/sin 超越函数）。
    // 缓冲区 32：当前拟合阶数为 10，留足余量。
    double Ta[32], Tb[32], dTa[32], dTb[32];
    chebyshev_basis(config.a_order, a_norm, Ta, dTa);
    chebyshev_basis(config.b_order, b_norm, Tb, dTb);

    // 四个偏导在同一个双重循环里累加：
    //   dPit/da = ΣΣ pit[i][j]·Tb[j]·dTa[i]·sa
    //   dPit/db = ΣΣ pit[i][j]·Ta[i]·dTb[j]·sb
    //   dRol/da = ΣΣ rol[i][j]·Tb[j]·dTa[i]·sa
    //   dRol/db = ΣΣ rol[i][j]·Ta[i]·dTb[j]·sb
    double dPit_da = 0.0, dPit_db = 0.0, dRol_da = 0.0, dRol_db = 0.0;
    for (int i = 0; i <= config.a_order; ++i) {
        const double tai = Ta[i];
        const double dtai_s = dTa[i] * sa;
        const double* cp = config.pit_coeffs[i].data();
        const double* cr = config.rol_coeffs[i].data();
        for (int j = 0; j <= config.b_order; ++j) {
            const double tbj = Tb[j];
            const double dtbj_s = dTb[j] * sb;
            dPit_da += cp[j] * tbj * dtai_s;
            dPit_db += cp[j] * tai * dtbj_s;
            dRol_da += cr[j] * tbj * dtai_s;
            dRol_db += cr[j] * tai * dtbj_s;
        }
    }

    jacob[0] = dPit_da;
    jacob[1] = dPit_db;
    jacob[2] = dRol_da;
    jacob[3] = dRol_db;
}

} // namespace parallel_ankle
