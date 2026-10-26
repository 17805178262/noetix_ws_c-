#include <cmath>

#include "Types.hpp"

#include "FK.hpp"
#include "IK.hpp"
#include "parallel_ankle.hpp"

namespace parallel_ankle {

ParallelAnkle::ParallelAnkle(int left_pitch, int left_roll, int right_pitch, int right_roll)
    : ankle_motor_ready_flag_(false) {
    // 加载 FK/FK_V_JB 使用的 Chebyshev 拟合系数。
    load_coeffs(chebyshev_config_);

    // 当前启用左右踝两组二自由度并联机构。
    //
    // 聚合初始化字段顺序必须和 ParallelPairConfig 完全一致：
    //   name,
    //   motor_1_idx, motor_2_idx,
    //   pitch_idx, roll_idx,
    //   pitch_sign, roll_sign。
    //
    // 这里把“真实电机索引”和“任务空间 pitch/roll 索引”分开写，
    // 目的就是避免以前 roll/pitch 互相写反的问题。
    pairs_ = {
    {
        "left_ankle",
        left_pitch,   // motor_1_idx (q1 / a)
        left_roll,    // motor_2_idx (q2 / b)
        left_pitch,   // pitch_idx
        left_roll,    // roll_idx
        1.0,  // pitch_sign
        -1.0  // roll_sign（左右腿镜像）
    },
    {
        "right_ankle",
        right_pitch,  // motor_1_idx (q1 / a)
        right_roll,   // motor_2_idx (q2 / b)
        right_pitch,  // pitch_idx
        right_roll,   // roll_idx
        1.0,  // pitch_sign
        1.0   // roll_sign
    }
    };

    pair_ready_.assign(pairs_.size(), 0);
    last_valid_.resize(pairs_.size());
    cached_jacob_.resize(pairs_.size());
    cached_motor_state_.resize(pairs_.size());

    // 数学库零位约定：平台水平时任务零位对应电机零位 a=b=0；
    // 电机硬件零位偏置已在 IK.cpp 内部用 MOTOR_PI_OFFSET_hi/low 扣除。
}

void ParallelAnkle::parallel_ankle_solve_state(std::vector<MotorData>& joint_data) {
    // 每帧重新评估每组机构的有效性，整体 flag 为各组的与。
    ankle_motor_ready_flag_ = true;

    for (size_t k = 0; k < pairs_.size(); ++k) {
        solve_state_pair(pairs_[k], k, joint_data);
        if (!pair_ready_[k]) {
            ankle_motor_ready_flag_ = false;
        }
    }
}

void ParallelAnkle::parallel_ankle_solve_cmd(std::vector<MotorData>& joint_data) {
    // 有效性由本帧先执行的 parallel_ankle_solve_state 给出，这里直接消费。
    for (size_t k = 0; k < pairs_.size(); ++k) {
        solve_cmd_pair(pairs_[k], k, joint_data);
    }
}

void ParallelAnkle::solve_state_pair(const ParallelPairConfig& pair,
                                     size_t pair_idx,
                                     std::vector<MotorData>& joint_data) {
    char& ready = pair_ready_[pair_idx];
    LastValidState& cache = last_valid_[pair_idx];
    ready = 0;

    // 从真实电机空间读取原始反馈（motor_idx 与 pitch/roll_idx 共用槽位，
    // 必须在写回任务空间之前读完）。
    const double q_1 = joint_data[pair.motor_1_idx].pos_;
    const double q_2 = joint_data[pair.motor_2_idx].pos_;
    const double dq_1 = joint_data[pair.motor_1_idx].vel_;
    const double dq_2 = joint_data[pair.motor_2_idx].vel_;
    const double tau_1 = joint_data[pair.motor_1_idx].tau_;
    const double tau_2 = joint_data[pair.motor_2_idx].tau_;

    // 只检查数值有效性，不用固定关节位置或拟合域阈值阻断真机启动。
    bool valid = std::isfinite(q_1) && std::isfinite(q_2) &&
                 std::isfinite(dq_1) && std::isfinite(dq_2) &&
                 std::isfinite(tau_1) && std::isfinite(tau_2);

    double q_pitch = 0.0;
    double q_roll = 0.0;
    double jacob_fk[4] = {0.0, 0.0, 0.0, 0.0};

    if (valid) {
        // 正运动学：真实电机角 q1/q2 -> 数学模型 pitch/roll。
        // Chebyshev 拟合给种子，再用精确解析 IK 做 2 次 Newton 抛光：
        // 一次调用同时产出抛光后的任务角、工作点正雅可比和电机空间往返残差。
        FKResult fk;
        FK_polished(q_1, q_2, fk, chebyshev_config_);
        q_pitch = fk.Pit;
        q_roll  = fk.Rol;
        jacob_fk[0] = fk.J[0];
        jacob_fk[1] = fk.J[1];
        jacob_fk[2] = fk.J[2];
        jacob_fk[3] = fk.J[3];

        // 正雅可比 J_fk = d(Pit,Rol)/d(a,b)，由 Chebyshev 拟合式解析求导
        // （上面 FK_polished 已在实测电机角处算好）。速度映射和两侧力矩映射
        // 都只需要这一个雅可比（虚功原理）：
        //   v_task = J_fk·v_motor，τ_motor = J_fk^T·τ_task，τ_task = J_fk^{-T}·τ_motor。

        // 任务空间变换前检查正解和雅可比的有限性。
        if (!std::isfinite(q_pitch) || !std::isfinite(q_roll) ||
            !std::isfinite(fk.res_a) || !std::isfinite(fk.res_b)) {
            valid = false;
        }
        for (const double v : jacob_fk) if (!std::isfinite(v)) valid = false;

        // 雅可比不可逆时无法计算速度和力矩，避免除零。
        const double det = jacob_fk[0] * jacob_fk[3] - jacob_fk[1] * jacob_fk[2];
        if (std::fabs(det) < 1e-6) {
            valid = false;
        }
    }

    if (!valid) {
        // 解算失败：任务空间保持上一帧有效值（首帧从未有效过则输出 0），
        // 不把原始电机角或外推垃圾值写给下游控制器。
        joint_data[pair.pitch_idx].pos_ = cache.valid ? cache.pitch_pos : 0.0;
        joint_data[pair.roll_idx].pos_  = cache.valid ? cache.roll_pos  : 0.0;
        joint_data[pair.pitch_idx].vel_ = cache.valid ? cache.pitch_vel : 0.0;
        joint_data[pair.roll_idx].vel_  = cache.valid ? cache.roll_vel  : 0.0;
        joint_data[pair.pitch_idx].tau_ = cache.valid ? cache.pitch_tau : 0.0;
        joint_data[pair.roll_idx].tau_  = cache.valid ? cache.roll_tau  : 0.0;
        // ready 保持 false；solve_cmd 会把该组两台电机的五项MIT命令清零。
        return;
    }

    // 速度映射：v_task = J_fk · v_motor（J_fk 已在有效性检查阶段算出）。
    const double dq_pitch = jacob_fk[0] * dq_1 + jacob_fk[1] * dq_2;
    const double dq_roll = jacob_fk[2] * dq_1 + jacob_fk[3] * dq_2;

    // 力矩映射（虚功原理）：τ_task = J_fk^{-T} · τ_motor。
    // 逆雅可比 J_ik = inv(J_fk) 直接对 2x2 矩阵解析求逆，
    // 不再对 IK 做有限差分——状态侧与命令侧因此严格共用同一个雅可比。
    //   J_fk = [j0 j1; j2 j3]，inv = 1/det [ j3 -j1; -j2  j0]
    const double det = jacob_fk[0] * jacob_fk[3] - jacob_fk[1] * jacob_fk[2];
    const double inv_det = 1.0 / det;
    const double ji0 =  jacob_fk[3] * inv_det;  // da/dPit
    const double ji1 = -jacob_fk[1] * inv_det;  // da/dRol
    const double ji2 = -jacob_fk[2] * inv_det;  // db/dPit
    const double ji3 =  jacob_fk[0] * inv_det;  // db/dRol

    const double tau_pitch = ji0 * tau_1 + ji2 * tau_2;
    const double tau_roll = ji1 * tau_1 + ji3 * tau_2;

    // 写回任务空间。
    // pitch 永远写 pitch_idx，roll 永远写 roll_idx；
    // 方向差异只通过 sign 处理，不再靠交换变量名硬凑。
    joint_data[pair.pitch_idx].pos_ = pair.pitch_sign * q_pitch;
    joint_data[pair.roll_idx].pos_ = pair.roll_sign * q_roll;

    joint_data[pair.pitch_idx].vel_ = pair.pitch_sign * dq_pitch;
    joint_data[pair.roll_idx].vel_ = pair.roll_sign * dq_roll;

    joint_data[pair.pitch_idx].tau_ = pair.pitch_sign * tau_pitch;
    joint_data[pair.roll_idx].tau_ = pair.roll_sign * tau_roll;

    // 缓存本帧雅可比（数学坐标），供同周期 solve_cmd 复用，避免重复求值。
    // 该雅可比由 FK_polished 在实测电机角 (q1,q2) 处一次性算出；抛光已使
    // IK(任务位姿) ≈ (q1,q2) 到 ~1e-9 rad（远优于旧的纯拟合 1e-4），
    // 故 cmd 侧工作点与 state 侧实测电机角几乎完全重合，直接复用即可，
    // 且 state/cmd 严格共用同一雅可比。
    cached_jacob_[pair_idx] = {jacob_fk[0], jacob_fk[1], jacob_fk[2], jacob_fk[3]};
    // 缓存覆盖任务空间槽位前读取的电机状态，供命令侧计算耦合项。
    cached_motor_state_[pair_idx] = {q_1, q_2, dq_1, dq_2};

    // 缓存本帧有效结果（公开符号值），供下一帧失败时保持。
    cache.valid = true;
    cache.pitch_pos = joint_data[pair.pitch_idx].pos_;
    cache.roll_pos = joint_data[pair.roll_idx].pos_;
    cache.pitch_vel = joint_data[pair.pitch_idx].vel_;
    cache.roll_vel = joint_data[pair.roll_idx].vel_;
    cache.pitch_tau = joint_data[pair.pitch_idx].tau_;
    cache.roll_tau = joint_data[pair.roll_idx].tau_;

    ready = 1;
}

void ParallelAnkle::solve_cmd_pair(const ParallelPairConfig& pair,
                                   size_t pair_idx,
                                   std::vector<MotorData>& joint_data) {
    // 失效时不能只清零前馈：电机内部的位置环仍会响应旧目标，必须清零五项。
    const auto clear_command = [&]() {
        for (const int index : {pair.motor_1_idx, pair.motor_2_idx}) {
            auto& motor = joint_data[index];
            motor.pos_des_ = 0.0;
            motor.vel_des_ = 0.0;
            motor.kp_ = 0.0;
            motor.kd_ = 0.0;
            motor.ff_ = 0.0;
        }
    };
    if (!pair_ready_[pair_idx]) {
        clear_command();
        return;
    }
    const auto& pitch = joint_data[pair.pitch_idx];
    const auto& roll = joint_data[pair.roll_idx];
    // 零增益且零前馈表示释放；不对无意义目标做逆解，也不遗留位置环增益。
    if (pitch.kp_ == 0.0 && pitch.kd_ == 0.0 && pitch.ff_ == 0.0 &&
        roll.kp_ == 0.0 && roll.kd_ == 0.0 && roll.ff_ == 0.0) {
        clear_command();
        return;
    }
    for (const int index : {pair.pitch_idx, pair.roll_idx}) {
        const auto& task = joint_data[index];
        if (!std::isfinite(task.pos_des_) || !std::isfinite(task.vel_des_) ||
            !std::isfinite(task.kp_) || !std::isfinite(task.kd_) ||
            !std::isfinite(task.ff_) || task.kp_ < 0.0 || task.kd_ < 0.0) {
            clear_command();
            return;
        }
    }

    // 力位混合控制：任务空间 PD 在当前工作点换算为电机空间位控，由电机内部
    // 位置环产生等效力矩（MIT 协议）：
    //   电机总输出 = kp·(q_des−q) + kd·(q̇_des−q̇) + ff
    //              = K_motor·δq + D_motor·δq̇ + J_fkᵀ·ff_task（局部线性等效）
    //
    // 1. 读取任务空间期望值与增益（公开坐标）。
    //    motor_idx 与 pitch/roll_idx 同槽：所有任务空间字段必须在写回电机
    //    命令之前读完，ff_ 尤其如此（第 6 步写回会覆盖该槽位）。
    const double pos_des_pitch_pub = joint_data[pair.pitch_idx].pos_des_;
    const double pos_des_roll_pub  = joint_data[pair.roll_idx].pos_des_;
    const double vel_des_pitch_pub = joint_data[pair.pitch_idx].vel_des_;
    const double vel_des_roll_pub  = joint_data[pair.roll_idx].vel_des_;
    const double ff_pitch_pub      = joint_data[pair.pitch_idx].ff_;
    const double ff_roll_pub       = joint_data[pair.roll_idx].ff_;
    const double kp_pitch = joint_data[pair.pitch_idx].kp_;
    const double kp_roll  = joint_data[pair.roll_idx].kp_;
    const double kd_pitch = joint_data[pair.pitch_idx].kd_;
    const double kd_roll  = joint_data[pair.roll_idx].kd_;

    // 2. 公开坐标 → 数学模型坐标（符号修正）。kp/kd 为标量增益，不乘符号。
    const double math_pos_des_pitch = pair.pitch_sign * pos_des_pitch_pub;
    const double math_pos_des_roll  = pair.roll_sign  * pos_des_roll_pub;
    const double math_vel_des_pitch = pair.pitch_sign * vel_des_pitch_pub;
    const double math_vel_des_roll  = pair.roll_sign  * vel_des_roll_pub;
    const double math_ff_pitch      = pair.pitch_sign * ff_pitch_pub;
    const double math_ff_roll       = pair.roll_sign  * ff_roll_pub;

    // 3. 位置：精确解析 IK 求电机目标角（禁止雅可比线性化近似）。
    double q1_des = 0.0, q2_des = 0.0;
    IK(math_pos_des_pitch, math_pos_des_roll, q1_des, q2_des);
    if (!std::isfinite(q1_des) || !std::isfinite(q2_des)) {
        clear_command();
        return;
    }

    // 雅可比复用本帧 solve_state 在实测电机角处算出的 J_fk（数学坐标）。
    const std::array<double, 4>& J = cached_jacob_[pair_idx];
    const double j0 = J[0], j1 = J[1], j2 = J[2], j3 = J[3];

    // 4. 速度：逆雅可比 v_motor = J_fk⁻¹ · v_des。
    const double det = j0 * j3 - j1 * j2;
    const double dq1_des = ( j3 * math_vel_des_pitch - j1 * math_vel_des_roll) / det;
    const double dq2_des = (-j2 * math_vel_des_pitch + j0 * math_vel_des_roll) / det;

    // 5. 刚度/阻尼相似变换：K_motor = Jᵀ·K_task·J，D_motor = Jᵀ·D_task·J。
    const double K00 = kp_pitch * j0 * j0 + kp_roll * j2 * j2;
    const double K11 = kp_pitch * j1 * j1 + kp_roll * j3 * j3;
    const double K01 = kp_pitch * j0 * j1 + kp_roll * j2 * j3;
    const double D00 = kd_pitch * j0 * j0 + kd_roll * j2 * j2;
    const double D11 = kd_pitch * j1 * j1 + kd_roll * j3 * j3;
    const double D01 = kd_pitch * j0 * j1 + kd_roll * j2 * j3;

    // 电机空间误差（solve_state 缓存的本帧实测电机状态，数学坐标）。
    const MotorState& ms = cached_motor_state_[pair_idx];
    const double q1_err  = q1_des  - ms.q1;
    const double q2_err  = q2_des  - ms.q2;
    const double dq1_err = dq1_des - ms.dq1;
    const double dq2_err = dq2_des - ms.dq2;

    // 6. 分解：对角元 → 电机 kp/kd；非对角耦合项 + Jᵀ·任务前馈 → ff。
    const double ff_1 = j0 * math_ff_pitch + j2 * math_ff_roll
                      + K01 * q2_err + D01 * dq2_err;
    const double ff_2 = j1 * math_ff_pitch + j3 * math_ff_roll
                      + K01 * q1_err + D01 * dq1_err;  // K10=K01, D10=D01

    // 混合控制的任一输出异常都要整组清零，避免向MIT编码器传入NaN/Inf。
    for (const double value : {q1_des, q2_des, dq1_des, dq2_des,
                               K00, K11, D00, D11, ff_1, ff_2}) {
        if (!std::isfinite(value)) {
            clear_command();
            return;
        }
    }

    // 7. 写回电机空间命令。
    joint_data[pair.motor_1_idx].pos_des_ = q1_des;
    joint_data[pair.motor_1_idx].vel_des_ = dq1_des;
    joint_data[pair.motor_1_idx].kp_ = K00;
    joint_data[pair.motor_1_idx].kd_ = D00;
    joint_data[pair.motor_1_idx].ff_ = ff_1;

    joint_data[pair.motor_2_idx].pos_des_ = q2_des;
    joint_data[pair.motor_2_idx].vel_des_ = dq2_des;
    joint_data[pair.motor_2_idx].kp_ = K11;
    joint_data[pair.motor_2_idx].kd_ = D11;
    joint_data[pair.motor_2_idx].ff_ = ff_2;
}

} // namespace parallel_ankle
