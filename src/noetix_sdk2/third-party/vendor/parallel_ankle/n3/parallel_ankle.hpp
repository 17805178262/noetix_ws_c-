#ifndef PARALLEL_ANKLE_HPP
#define PARALLEL_ANKLE_HPP

#include <array>
#include <vector>
#include "coeffs.hpp"
#include "Types.hpp"

namespace parallel_ankle {

/**
 * @brief 一组二自由度并联机构的索引和符号配置。
 *
 * 一个 ParallelPairConfig 描述一组二自由度并联机构（左踝、右踝等）：
 * 两台真实电机（motor_1/motor_2）对应数学库里的电机角 a/b（即 q1/q2），
 * 两个对外暴露的任务空间关节（pitch/roll）对应数学库里的 Pit/Rol。
 *
 * 这样做的目的，是把原来散落在代码里的魔法数字集中起来：
 * - 不在算法里硬写 joint index；
 * - 不在算法里硬写 roll/pitch 的交换关系；
 * - 不在算法里硬写左右腿或者脖子的符号差异。
 *
 * 注意：这个结构体使用聚合初始化，字段顺序非常重要。
 * 初始化时推荐按下面顺序写，并且每一行加注释，防止把索引和符号填串。
 */
struct ParallelPairConfig {
    // 仅用于日志和调试，例如 "left_ankle"、"right_ankle"。
    const char* name;

    // 并联机构的两个真实电机在 joint_data 里的索引。
    // 数学库里的 IK/FK 使用 q1、q2，这里对应 motor_1_idx、motor_2_idx。
    int motor_1_idx;
    int motor_2_idx;

    // 对外暴露的任务空间关节索引。
    // pitch_idx 表示 pitch 这个任务轴写回/读取 joint_data 的位置。
    // roll_idx 表示 roll 这个任务轴写回/读取 joint_data 的位置。
    int pitch_idx;
    int roll_idx;

    // 任务空间符号修正。
    // 用于处理机械镜像、坐标系定义差异（例如左右腿 roll 方向相反）。
    // roll_sign = -1.0 表示算法算出的 roll 写回系统时需要取反。
    double pitch_sign;
    double roll_sign;
};

/**
 * @brief 并联机构求解适配层。
 *
 * 这个类本身不实现具体的并联机构数学模型，真正的 FK/IK/Jacobian 来自 parallel_solve 数学库。
 * 它负责把 N3 的 joint_data 映射到数学库需要的 q1/q2、pitch/roll，再把结果写回。
 *
 * 公开接口：
 * - parallel_ankle_solve_state：把电机反馈状态转换成 pitch/roll 状态；
 * - parallel_ankle_solve_cmd：把 pitch/roll 命令转换成电机 MIT 力位混合命令。
 *
 * 内部通过 pairs_ 支持多组机构（当前为左踝、右踝两组）。
 *
 * 失效保护：
 * - 仅检查非有限数值和不可逆雅可比，不用固定电机角限值拦截启动；
 * - 数值无效时，该组任务空间状态保持上一帧有效值（首帧未有效过则为 0），
 *   对应的 pair_ready_ 置 false；
 * - solve_cmd 消费 pair_ready_，无效组的两台电机目标、增益和前馈全部清零。
 */
class ParallelAnkle {
public:
    ParallelAnkle(int left_pitch, int left_roll, int right_pitch, int right_roll);

    // 状态方向：真实电机空间 -> 任务空间。
    // 典型流程：
    // joint_data[motor_1/motor_2].pos_/vel_/tau_ -> FK/Jacobian ->
    // joint_data[pitch/roll].pos_/vel_/tau_。
    void parallel_ankle_solve_state(std::vector<MotorData>& joint_data);

    // 命令方向：任务空间 -> 真实电机空间。
    // 典型流程：
    // joint_data[pitch/roll].pos_des_/vel_des_/kp_/kd_/ff_ -> IK/Jacobian ->
    // joint_data[motor_1/motor_2].pos_des_/vel_des_/kp_/kd_/ff_。
    void parallel_ankle_solve_cmd(std::vector<MotorData>& joint_data);

    // 只要任意一组机构出现非有限数值或不可逆雅可比，这个 flag 就为 false。
    bool is_motor_ready() const { return ankle_motor_ready_flag_; }

private:
    // 单组机构最近一次有效的任务空间状态（写回 joint_data 的公开符号值）。
    // 解算失败时用它保持上一帧，避免把原始电机角或外推垃圾值写给下游控制器。
    struct LastValidState {
        bool valid = false;
        double pitch_pos = 0.0;
        double roll_pos = 0.0;
        double pitch_vel = 0.0;
        double roll_vel = 0.0;
        double pitch_tau = 0.0;
        double roll_tau = 0.0;
    };

    // 处理单组机构的状态正解。
    // 所有索引、符号都必须从 pair 读取，避免再出现硬编码的 roll/pitch 搞反问题。
    void solve_state_pair(const ParallelPairConfig& pair,
                          size_t pair_idx,
                          std::vector<MotorData>& joint_data);

    // 处理单组机构的命令逆解。
    // 和 solve_state_pair 使用同一个 pair 配置，保证状态方向和命令方向的映射互逆。
    // pair_ready_[pair_idx] 为 false 时直接把该组两台电机的五项命令清零。
    void solve_cmd_pair(const ParallelPairConfig& pair,
                        size_t pair_idx,
                        std::vector<MotorData>& joint_data);

    // 当前启用的所有并联机构配置。
    std::vector<ParallelPairConfig> pairs_;

    // 每组机构当前帧是否有效（solve_state 写入，solve_cmd 消费）。
    // 用 char 而不是 bool：std::vector<bool> 的 operator[] 返回位代理，无法绑定 bool&。
    std::vector<char> pair_ready_;

    // 每组机构最近一次有效的任务空间状态缓存。
    std::vector<LastValidState> last_valid_;

    // 每组机构本帧 solve_state 在当前电机角处算出的正雅可比 J_fk（行主序 2x2，
    // 数学坐标，布局见 Jacobian.hpp）。solve_cmd 在同周期内直接复用，
    // 省去重复的 IK + FK_V_JB；仅在该组 pair_ready_ 为真时有效。
    std::vector<std::array<double, 4>> cached_jacob_;

    // 状态正解会覆盖电机槽位；命令侧的耦合补偿需要同帧原始电机角和角速度。
    struct MotorState {
        double q1 = 0.0;
        double q2 = 0.0;
        double dq1 = 0.0;
        double dq2 = 0.0;
    };
    std::vector<MotorState> cached_motor_state_;

    // 并联机构整体安全状态。
    // true 表示所有 pair 当前帧都通过有效性检查；false 表示至少一组无效。
    bool ankle_motor_ready_flag_;

    // Chebyshev 拟合系数配置，供 FK/FK_V_JB 使用。
    // 构造函数里通过 load_coeffs 初始化。
    ChebyshevConfig chebyshev_config_;
};

} // namespace parallel_ankle

#endif // PARALLEL_ANKLE_HPP
