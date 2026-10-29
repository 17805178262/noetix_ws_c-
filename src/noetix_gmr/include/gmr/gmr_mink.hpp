#pragma once
/**
 * gmr_mink.hpp — C++ port of Python GMR + mink IK
 */
#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

extern "C" {
#include "third_party/daqp/include/api.h"
#include "third_party/daqp/include/types.h"
}

#include "gmr/body_map.hpp"
#include "gmr/foot_contact.hpp"
#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <cctype>
#include <fstream>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <limits>
#include <chrono>
#include <memory>

namespace gmr_mink {

using BodyData = gmr::BodyData;
using BodyMap  = gmr::BodyMap;

namespace quat {
inline Eigen::Vector4d normalise(const Eigen::Vector4d& q) {
    double n = q.norm(); return n < 1e-12 ? Eigen::Vector4d(1,0,0,0) : q/n;
}
inline Eigen::Vector4d conjugate(const Eigen::Vector4d& q) {
    return {q[0],-q[1],-q[2],-q[3]};
}
inline Eigen::Vector4d multiply(const Eigen::Vector4d& a, const Eigen::Vector4d& b) {
    return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
inline Eigen::Vector3d rotate(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
    Eigen::Vector4d qv{0,v.x(),v.y(),v.z()};
    auto r = multiply(multiply(q,qv),conjugate(q));
    return {r[1],r[2],r[3]};
}
inline Eigen::Vector3d so3_error(const Eigen::Vector4d& q_cur, const Eigen::Vector4d& q_tgt) {
    auto qr = normalise(multiply(conjugate(q_cur), q_tgt));
    if (qr[0] < 0) qr = -qr;
    double w = std::clamp(qr[0], -1.0, 1.0);
    double angle = 2.0 * std::acos(w);
    double s = std::sqrt(std::max(1e-12, 1.0-w*w));
    return Eigen::Vector3d{qr[1]/s, qr[2]/s, qr[3]/s} * angle;
}
} // namespace quat

struct IKEntry {
    std::string     robot_body;
    std::string     human_body;
    int            body_id = -1;
    bool           upper_body = false;
    double          pos_weight = 0.0;
    double          rot_weight = 0.0;
    Eigen::Vector3d pos_offset = Eigen::Vector3d::Zero();
    Eigen::Vector4d rot_offset = Eigen::Vector4d(1,0,0,0);
};

class GMR {
public:
    GMR(const std::string& xml_path,
        const std::string& ik_config_path,
        double actual_human_height = 1.8,
        double damping = 0.5,
        bool   verbose = false)
        : damping_(damping), verbose_(verbose)
    {
        char err[1024] = {};
        // 配置解析失败时也释放已加载的 MuJoCo 资源。
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_owner(
            mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err)), mj_deleteModel);
        model_ = model_owner.get();
        if (!model_) throw std::runtime_error(std::string("mj_loadXML: ")+err);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data_owner(mj_makeData(model_), mj_deleteData);
        data_ = data_owner.get();
        if (!data_) throw std::runtime_error("MuJoCo data allocation failed");
        nv_ = model_->nv;
        nq_ = model_->nq;

        std::ifstream f(ik_config_path);
        if (!f) throw std::runtime_error("Cannot open: "+ik_config_path);
        nlohmann::json j; f >> j;

        human_root_name_ = j["human_root_name"].get<std::string>();
        sim2real_profile_ = j.value("solver_profile", std::string{}) == "sim2real";
        ground_height_   = j["ground_height"].get<double>();
        foot_target_height_ = j.value("foot_target_height", 0.06);
        robot_ground_height_ = j.value("robot_ground_height", 0.0);
        align_robot_sole_ = j.value("align_robot_sole", false);
        adaptive_grounding_ = j.value("adaptive_grounding", false);
        ground_reference_alpha_ = std::clamp(j.value("ground_reference_alpha", 0.01), 0.0, 1.0);
        ground_takeoff_step_ = std::max(0.0, j.value("ground_takeoff_step", 0.008));
        ground_clearance_deadband_ = std::max(0.0, j.value("ground_clearance_deadband", 0.01));
        lock_planted_feet_ = j.value("lock_planted_feet", false);
        if (sim2real_profile_) {
            align_robot_sole_ = false;
            lock_planted_feet_ = false;
        }
        foot_contact_config_.height = foot_target_height_;
        foot_contact_config_.enter_height = j.value("foot_contact_enter_height", 0.025);
        foot_contact_config_.exit_height = j.value("foot_contact_exit_height", 0.055);
        foot_contact_config_.enter_speed = j.value("foot_contact_enter_speed", 0.20);
        foot_contact_config_.exit_speed = j.value("foot_contact_exit_speed", 0.45);
        foot_contact_config_.release_time = j.value("foot_contact_release_time", 0.05);
        foot_contact_config_.max_displacement = j.value("foot_contact_max_displacement", 0.06);
        foot_contact_config_.max_rotation = j.value("foot_contact_max_rotation", 0.35);
        if (lock_planted_feet_) (void)gmr::FootContact(foot_contact_config_);
        const auto upper_body = j.value("upper_body_root", std::string());
        if (!upper_body.empty()) {
            upper_body_root_ = mj_name2id(model_, mjOBJ_BODY, upper_body.c_str());
            if (upper_body_root_ <= 0 || model_->body_parentid[upper_body_root_] == 0)
                throw std::invalid_argument("upper_body_root must name a subtree below the floating base");
        }
        upper_body_dofs_.resize(nv_, false);
        for (int v = 0; v < nv_; ++v)
            upper_body_dofs_[v] = inUpperBody(model_->jnt_bodyid[model_->dof_jntid[v]]);
        double ratio     = actual_human_height / j["human_height_assumption"].get<double>();
        for (auto& [k,v] : j["human_scale_table"].items())
            human_scale_table_[k] = v.get<double>() * ratio;

        use_table1_ = j.value("use_ik_match_table1", false);
        use_table2_ = j.value("use_ik_match_table2", false);

        auto parse = [&](const std::string& key,
                         std::vector<IKEntry>& entries,
                         std::map<std::string,Eigen::Vector3d>& pos_off,
                         std::map<std::string,Eigen::Vector4d>& rot_off) {
            if (!j.contains(key)) return;
            for (auto& [frame_name, entry] : j[key].items()) {
                IKEntry e;
                e.robot_body = frame_name;
                e.human_body = entry[0].get<std::string>();
                e.body_id = mj_name2id(model_, mjOBJ_BODY, frame_name.c_str());
                e.upper_body = inUpperBody(e.body_id);
                e.pos_weight = entry[1].get<double>();
                e.rot_weight = entry[2].get<double>();
                e.pos_offset = {entry[3][0].get<double>(),
                                entry[3][1].get<double>(),
                                entry[3][2].get<double>()};
                e.rot_offset = {entry[4][0].get<double>(),
                                entry[4][1].get<double>(),
                                entry[4][2].get<double>(),
                                entry[4][3].get<double>()};
                pos_off[e.human_body] = e.pos_offset - Eigen::Vector3d(0,0,ground_height_);
                rot_off[e.human_body] = e.rot_offset;
                if (e.pos_weight != 0 || e.rot_weight != 0)
                    entries.push_back(e);
            }
        };
        parse("ik_match_table1", entries1_, pos_offsets1_, rot_offsets1_);
        parse("ik_match_table2", entries2_, pos_offsets2_, rot_offsets2_);
        // 新Pico路径按原包做全身联合求解，其他路径继续使用原有上半身独立求解。
        if (!sim2real_profile_) {
            auto split_upper = [](std::vector<IKEntry>& entries, std::vector<IKEntry>& upper) {
                for (const auto& entry : entries) if (entry.upper_body) upper.push_back(entry);
                entries.erase(std::remove_if(entries.begin(), entries.end(),
                    [](const IKEntry& entry) { return entry.upper_body; }), entries.end());
            };
            split_upper(entries1_, upper_entries1_);
            split_upper(entries2_, upper_entries2_);
        }

        buildVelBounds();

        if (verbose_)
            std::cout << "[GMR] nq=" << nq_ << " nv=" << nv_
                      << " vel_bounds=" << vel_bounds_.size()
                      << " t1=" << entries1_.size()
                      << " t2=" << entries2_.size() << "\n";

        mj_resetData(model_, data_);
        mj_forward(model_, data_);
        model_owner.release();
        data_owner.release();
    }

    ~GMR() {
        if (data_)  mj_deleteData(data_);
        if (model_) mj_deleteModel(model_);
    }
    GMR(const GMR&) = delete;
    GMR& operator=(const GMR&) = delete;

    Eigen::VectorXd retarget(const BodyMap& human_data, bool offset_to_ground = false,
                            double frame_time = std::numeric_limits<double>::quiet_NaN())
    {
        if (!std::isfinite(frame_time))
            frame_time = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        if (sim2real_profile_ && offset_to_ground)
            throw std::invalid_argument("offset_to_ground is incompatible with sim2real profile");
        BodyMap scaled = scaleHumanData(human_data);
        auto targets = [&](const auto& positions, const auto& rotations, auto& contacts, bool update_ground) {
            BodyMap result = applyGroundOffset(offsetHumanData(scaled, positions, rotations));
            if (offset_to_ground) result = offsetToGround(result, update_ground);
            if (lock_planted_feet_) stabilizePlantedFeet(result, frame_time, contacts);
            return result;
        };
        BodyMap offset = targets(pos_offsets1_, rot_offsets1_, foot_contacts1_, true);
        if (sim2real_profile_) {
            // 两阶段共用第一张表生成的目标，不做逐帧贴地、脚锁定或额外根节点修正。
            scaled_human_data_ = offset;
            if (use_table1_) solveSim2realStage(offset, entries1_);
            if (use_table2_) solveSim2realStage(offset, entries2_);
            Eigen::VectorXd out(nq_);
            for (int i = 0; i < nq_; ++i) out[i] = data_->qpos[i];
            return out;
        }
        // 两阶段使用各自的目标偏移；此前第二阶段实际上沿用了第一阶段的偏移。
        BodyMap offset2;
        if (use_table2_) offset2 = targets(pos_offsets2_, rot_offsets2_, foot_contacts2_, false);
        scaled_human_data_ = use_table1_ ? offset : offset2;

        if (frame_count_ == 0) {
            auto it = scaled_human_data_.find(human_root_name_);
            if (it != scaled_human_data_.end()) {
                const Eigen::Vector4d& hq = it->second.rot_wxyz;
                double n = hq.norm();
                Eigen::Vector4d q = (n > 1e-6) ? hq/n : Eigen::Vector4d(1,0,0,0);
                double siny = 2.0*(q[0]*q[3] + q[1]*q[2]);
                double cosy = 1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]);
                double yaw  = std::atan2(siny, cosy);
                data_->qpos[3] = std::cos(yaw*0.5);
                data_->qpos[4] = 0.0;
                data_->qpos[5] = 0.0;
                data_->qpos[6] = std::sin(yaw*0.5);
                mj_forward(model_, data_);
            }
        }
        frame_count_++;
        if (use_table1_) {
            solveIK(offset, entries1_);
            solveIK(offset, upper_entries1_);
        }
        if (use_table2_) {
            solveIK(offset2, entries2_);
            solveIK(offset2, upper_entries2_);
        }

        // Target-space grounding alone cannot guarantee that the robot sole
        // stays above the floor: joint limits and competing pelvis targets can
        // leave a residual IK error.  Correct the free root from the actual
        // foot geometry after IK so crouching/kneeling cannot push the visual (and
        // published) robot through the ground.
        if (offset_to_ground && align_robot_sole_)
            alignRobotFeetToGround(robot_ground_height_ +
                                   (adaptive_grounding_ ? tracked_ground_clearance_ : 0.0));

        Eigen::VectorXd out(nq_);
        for (int i = 0; i < nq_; ++i) out[i] = data_->qpos[i];
        return out;
    }

    const mjModel* model() const { return model_; }
    // 返回是否启用Pico的sim2real预处理与求解路径。
    bool sim2realProfile() const { return sim2real_profile_; }

    const BodyMap& getScaledHumanData() const { return scaled_human_data_; }
    void setGroundOffset(double g) { ground_offset_ = g; }

private:
    mjModel* model_ = nullptr;
    mjData*  data_  = nullptr;
    int nv_, nq_;
    double damping_;
    bool   verbose_;

    std::string human_root_name_;
    double ground_height_ = 0.0;
    double ground_offset_ = 0.0;
    double foot_target_height_ = 0.06;
    double robot_ground_height_ = 0.0;
    bool align_robot_sole_ = false;
    bool adaptive_grounding_ = false;
    double ground_reference_alpha_ = 0.01;
    double ground_takeoff_step_ = 0.008;
    double ground_clearance_deadband_ = 0.01;
    // 记录人体脚底的地面参考，双脚离地时保留真实抬升，不逐帧拉回地面。
    bool have_ground_reference_ = false;
    bool feet_airborne_ = false;
    double ground_reference_z_ = 0.0;
    double previous_lowest_z_ = 0.0;
    double tracked_ground_clearance_ = 0.0;
    bool lock_planted_feet_ = false;
    bool sim2real_profile_ = false;
    gmr::FootContact::Config foot_contact_config_;
    bool use_table1_ = false, use_table2_ = false;

    std::map<std::string,double>          human_scale_table_;
    std::map<std::string,Eigen::Vector3d> pos_offsets1_, pos_offsets2_;
    std::map<std::string,Eigen::Vector4d> rot_offsets1_, rot_offsets2_;
    std::vector<IKEntry> entries1_, entries2_;
    std::vector<IKEntry> upper_entries1_, upper_entries2_;
    int upper_body_root_ = -1;
    std::vector<bool> upper_body_dofs_;
    BodyMap scaled_human_data_;

    int frame_count_ = 0;

    std::map<std::string, gmr::FootContact> foot_contacts1_, foot_contacts2_;

    bool inUpperBody(int body) const {
        if (upper_body_root_ < 0) return false;
        for (; body > 0; body = model_->body_parentid[body])
            if (body == upper_body_root_) return true;
        return false;
    }

    struct VelBound { int vadr; int qadr; double lo; double hi; };
    std::vector<VelBound> vel_bounds_;

    void buildVelBounds() {
        vel_bounds_.clear();
        for (int i = 0; i < model_->njnt; ++i) {
            if (!model_->jnt_limited[i]) continue;
            int jtype = model_->jnt_type[i];
            if (jtype != mjJNT_HINGE && jtype != mjJNT_SLIDE) continue;
            VelBound b;
            b.qadr = model_->jnt_qposadr[i];
            b.vadr = model_->jnt_dofadr[i];
            b.lo   = model_->jnt_range[i*2+0];
            b.hi   = model_->jnt_range[i*2+1];
            vel_bounds_.push_back(b);
        }
    }

    BodyMap scaleHumanData(const BodyMap& src) const {
        auto rit = src.find(human_root_name_);
        if (rit == src.end()) return src;
        const Eigen::Vector3d root_pos = rit->second.position;
        double rs = human_scale_table_.count(human_root_name_)
                    ? human_scale_table_.at(human_root_name_) : 1.0;
        Eigen::Vector3d scaled_root = root_pos * rs;
        BodyMap out;
        for (auto& [name, bd] : src) {
            if (!human_scale_table_.count(name)) continue;
            double s = human_scale_table_.at(name);
            if (name == human_root_name_)
                out[name] = {scaled_root, bd.rot_wxyz};
            else
                out[name] = {(bd.position-root_pos)*s + scaled_root, bd.rot_wxyz};
        }
        return out;
    }

    BodyMap offsetHumanData(const BodyMap& src,
                            const std::map<std::string,Eigen::Vector3d>& pos_off,
                            const std::map<std::string,Eigen::Vector4d>& rot_off) const {
        BodyMap out;
        for (auto& [name, bd] : src) {
            Eigen::Vector3d pos = bd.position;
            Eigen::Vector4d rot = bd.rot_wxyz;
            if (rot_off.count(name))
                rot = quat::normalise(quat::multiply(rot, rot_off.at(name)));
            if (pos_off.count(name))
                pos += quat::rotate(rot, pos_off.at(name));
            out[name] = {pos, rot};
        }
        return out;
    }

    BodyMap applyGroundOffset(BodyMap src) const {
        for (auto& [name,bd] : src) bd.position[2] -= ground_offset_;
        return src;
    }

    // 两阶段共用一次地面状态更新，避免同一输入帧被重复计入地面漂移估计。
    BodyMap offsetToGround(BodyMap src, bool update_ground) {
        double lowest = std::numeric_limits<double>::infinity();
        for (const auto& [name, bd] : src)
            if (isFootTarget(name)) lowest = std::min(lowest, bd.position[2]);
        if (!std::isfinite(lowest)) return src;
        if (!adaptive_grounding_) {
            for (auto& [name, bd] : src)
                bd.position[2] = bd.position[2] - lowest + foot_target_height_;
            return src;
        }

        if (update_ground || !have_ground_reference_) {
            if (!have_ground_reference_) {
                ground_reference_z_ = lowest;
                previous_lowest_z_ = lowest;
                have_ground_reference_ = true;
            }
            const double rise = lowest - previous_lowest_z_;
            const double height = lowest - ground_reference_z_;
            if (feet_airborne_) {
                if (height <= foot_contact_config_.enter_height && rise <= ground_takeoff_step_)
                    feet_airborne_ = false;
            } else if (rise > ground_takeoff_step_ || height > foot_contact_config_.exit_height) {
                feet_airborne_ = true;
            }
            // 下移的地面立即跟随；向上的缓慢漂移低速校正，起跳期间冻结地面。
            if (!feet_airborne_) {
                if (lowest < ground_reference_z_) ground_reference_z_ = lowest;
                else ground_reference_z_ += ground_reference_alpha_ * (lowest - ground_reference_z_);
            }
            previous_lowest_z_ = lowest;
            // 连续死区避免起落地状态切换时，根节点瞬间跳过检测阈值。
            tracked_ground_clearance_ = std::max(0.0, lowest - ground_reference_z_ -
                                                     ground_clearance_deadband_);
        }
        for (auto& [name, bd] : src)
            bd.position[2] = bd.position[2] - ground_reference_z_ + foot_target_height_;
        return src;
    }

    bool isFootTarget(const std::string& name) const {
        std::string lo = name;
        std::transform(lo.begin(), lo.end(), lo.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return lo.find("foot") != std::string::npos ||
               lo.find("toe") != std::string::npos ||
               lo.find("ankle") != std::string::npos;
    }

    void stabilizePlantedFeet(BodyMap& targets, double frame_time,
                             std::map<std::string, gmr::FootContact>& contacts) {
        for (auto& [name, body] : targets) {
            if (!isFootTarget(name)) continue;
            contacts.try_emplace(name, foot_contact_config_).first->second.apply(body, frame_time);
        }
    }

    void alignRobotFeetToGround(double target_sole_z) {
        if (nq_ < 3 || model_->njnt == 0 || model_->jnt_type[0] != mjJNT_FREE)
            return;

        std::vector<int> foot_bodies;
        auto add_from = [&](const std::vector<IKEntry>& entries) {
            for (const auto& entry : entries) {
                if (!isFootTarget(entry.human_body)) continue;
                int bid = mj_name2id(model_, mjOBJ_BODY, entry.robot_body.c_str());
                if (bid >= 0 && std::find(foot_bodies.begin(), foot_bodies.end(), bid)
                                == foot_bodies.end())
                    foot_bodies.push_back(bid);
            }
        };
        add_from(entries1_);
        add_from(entries2_);

        double sole_z = std::numeric_limits<double>::infinity();
        for (int gid = 0; gid < model_->ngeom; ++gid) {
            if (std::find(foot_bodies.begin(), foot_bodies.end(),
                          model_->geom_bodyid[gid]) == foot_bodies.end())
                continue;

            const mjtNum* gp = data_->geom_xpos + 3 * gid;
            const mjtNum* gr = data_->geom_xmat + 9 * gid;
            const int mesh_id = model_->geom_dataid[gid];
            if (model_->geom_type[gid] == mjGEOM_MESH && mesh_id >= 0) {
                const int first = model_->mesh_vertadr[mesh_id];
                const int count = model_->mesh_vertnum[mesh_id];
                for (int i = 0; i < count; ++i) {
                    const float* v = model_->mesh_vert + 3 * (first + i);
                    const double z = gp[2] + gr[6] * v[0]
                                           + gr[7] * v[1]
                                           + gr[8] * v[2];
                    sole_z = std::min(sole_z, z);
                }
            } else if (model_->geom_type[gid] == mjGEOM_CAPSULE) {
                // A capsule's local Z half-length contributes only its vertical
                // projection. rbound also includes horizontal length, which
                // lifts the new N3's long foot capsules about 10 cm too high.
                const mjtNum* size = model_->geom_size + 3 * gid;
                const double extent = size[0] + std::abs(gr[8]) * size[1];
                sole_z = std::min(sole_z, double(gp[2] - extent));
            } else if (model_->geom_type[gid] == mjGEOM_SPHERE) {
                sole_z = std::min(sole_z, double(gp[2] - model_->geom_size[3 * gid]));
            } else {
                // Conservative fallback for primitive foot geoms.  rbound is
                // orientation-independent and therefore never allows sinking.
                sole_z = std::min(sole_z, double(gp[2] - model_->geom_rbound[gid]));
            }
        }

        if (std::isfinite(sole_z)) {
            data_->qpos[2] += target_sole_z - sole_z;
            mj_forward(model_, data_);
        }
    }

    void clampJoints() {
        for (auto& b : vel_bounds_)
            data_->qpos[b.qadr] = std::clamp(data_->qpos[b.qadr], b.lo, b.hi);
    }

    // 构造向量的反对称矩阵，用于旋转和位姿的李代数运算。
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d out;
        out << 0.0, -v.z(), v.y(),
               v.z(), 0.0, -v.x(),
              -v.y(), v.x(), 0.0;
        return out;
    }

    // 将旋转矩阵转为最短旋转向量。
    static Eigen::Vector3d so3Log(const Eigen::Matrix3d& rotation) {
        Eigen::Quaterniond q(rotation);
        q.normalize();
        if (q.w() < 0.0) q.coeffs() *= -1.0;
        Eigen::AngleAxisd aa(q);
        return aa.axis() * aa.angle();
    }

    // 计算SO(3)左雅可比的逆，小角度使用级数避免数值消减。
    static Eigen::Matrix3d so3LeftJacobianInverse(const Eigen::Vector3d& omega) {
        const double theta = omega.norm();
        const double theta2 = theta * theta;
        double beta;
        if (theta < 1e-4) {
            beta = (1.0 / 12.0) *
                (1.0 + theta2 / 60.0 *
                (1.0 + theta2 / 42.0 * (1.0 + theta2 / 40.0)));
        } else {
            beta = (1.0 / theta2) *
                (1.0 - 0.5 * theta / std::tan(0.5 * theta));
        }
        const Eigen::Matrix3d W = skew(omega);
        return Eigen::Matrix3d::Identity() - 0.5 * W + beta * W * W;
    }

    // 将相对位姿转为六维平移与旋转误差。
    static Eigen::Matrix<double,6,1> se3Log(const Eigen::Matrix3d& rotation,
                                            const Eigen::Vector3d& translation) {
        Eigen::Matrix<double,6,1> tangent;
        const Eigen::Vector3d omega = so3Log(rotation);
        const double theta2 = omega.squaredNorm();
        const Eigen::Matrix3d W = skew(omega);
        Eigen::Matrix3d vinv;
        if (theta2 < 1e-10) {
            vinv = Eigen::Matrix3d::Identity() - 0.5 * W + W * W / 12.0;
        } else {
            const double theta = std::sqrt(theta2);
            const double half = 0.5 * theta;
            vinv = Eigen::Matrix3d::Identity() - 0.5 * W
                + (1.0 - 0.5 * theta * std::cos(half) / std::sin(half))
                / theta2 * W * W;
        }
        tangent.head<3>() = vinv * translation;
        tangent.tail<3>() = omega;
        return tangent;
    }

    // 计算SE(3)雅可比中的平移与旋转耦合块。
    static Eigen::Matrix3d se3Q(const Eigen::Matrix<double,6,1>& tangent) {
        const double theta = tangent.tail<3>().norm();
        const double theta2 = theta * theta;
        double B, C, D;
        if (theta2 < 1e-10) {
            B = 1.0 / 6.0 + theta2 / 120.0;
            C = -1.0 / 24.0 + theta2 / 720.0;
            D = -1.0 / 60.0;
        } else {
            const double theta4 = theta2 * theta2;
            B = (theta - std::sin(theta)) / (theta2 * theta);
            C = (1.0 - 0.5 * theta2 - std::cos(theta)) / theta4;
            D = (2.0 * theta - 3.0 * std::sin(theta)
                + theta * std::cos(theta)) / (2.0 * theta4 * theta);
        }
        const Eigen::Matrix3d V = skew(tangent.head<3>());
        const Eigen::Matrix3d W = skew(tangent.tail<3>());
        const Eigen::Matrix3d VW = V * W;
        const Eigen::Matrix3d WV = VW.transpose();
        const Eigen::Matrix3d WVW = WV * W;
        const Eigen::Matrix3d VWW = VW * W;
        return 0.5 * V + B * (WV + VW + WVW)
            - C * (VWW - VWW.transpose() - 3.0 * WVW)
            + D * (WVW * W + W * WVW);
    }

    // 计算六维位姿误差的左雅可比逆。
    static Eigen::Matrix<double,6,6>
    se3LeftJacobianInverse(const Eigen::Matrix<double,6,1>& tangent) {
        Eigen::Matrix<double,6,6> result =
            Eigen::Matrix<double,6,6>::Identity();
        if (tangent.tail<3>().squaredNorm() < 1e-10)
            return result;
        const Eigen::Matrix3d Jinv = so3LeftJacobianInverse(tangent.tail<3>());
        result.topLeftCorner<3,3>() = Jinv;
        result.bottomRightCorner<3,3>() = Jinv;
        result.topRightCorner<3,3>() = -Jinv * se3Q(tangent) * Jinv;
        return result;
    }

    // 在机器人目标连杆的局部坐标系中计算人体目标位姿误差。
    Eigen::Matrix<double,6,1> sim2realError(
        const BodyData& target, int body_id) const {
        Eigen::Matrix3d R_current;
        const mjtNum* xm = data_->xmat + body_id * 9;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) R_current(r,c) = xm[r*3+c];
        const Eigen::Vector3d p_current(
            data_->xpos[body_id*3], data_->xpos[body_id*3+1], data_->xpos[body_id*3+2]);
        const Eigen::Quaterniond q_target(
            target.rot_wxyz[0], target.rot_wxyz[1],
            target.rot_wxyz[2], target.rot_wxyz[3]);
        const Eigen::Matrix3d R_target = q_target.normalized().toRotationMatrix();
        return se3Log(R_current.transpose() * R_target,
                      R_current.transpose() * (target.position - p_current));
    }

    // 构建加权位姿误差QP，解出本轮关节增量并限制在当前模型的关节范围内。
    void runSim2realStep(const BodyMap& targets, const std::vector<IKEntry>& entries) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(nv_, nv_) * damping_;
        Eigen::VectorXd c = Eigen::VectorXd::Zero(nv_);

        for (const auto& entry : entries) {
            const auto target = targets.find(entry.human_body);
            if (target == targets.end() || entry.body_id < 0) continue;
            const int bid = entry.body_id;

            Eigen::Matrix<double,3,Eigen::Dynamic,Eigen::RowMajor> jp_world(3,nv_), jr_world(3,nv_);
            mj_jacBody(model_, data_, jp_world.data(), jr_world.data(), bid);
            Eigen::Matrix3d R_current;
            const mjtNum* xm = data_->xmat + bid * 9;
            for (int r = 0; r < 3; ++r)
                for (int col = 0; col < 3; ++col) R_current(r,col) = xm[r*3+col];
            Eigen::Matrix<double,6,Eigen::Dynamic> body_jacobian(6, nv_);
            body_jacobian.topRows<3>() = R_current.transpose() * jp_world;
            body_jacobian.bottomRows<3>() = R_current.transpose() * jr_world;

            const Eigen::Quaterniond q_target(
                target->second.rot_wxyz[0], target->second.rot_wxyz[1],
                target->second.rot_wxyz[2], target->second.rot_wxyz[3]);
            const Eigen::Matrix3d R_target = q_target.normalized().toRotationMatrix();
            const Eigen::Vector3d p_current(
                data_->xpos[bid*3], data_->xpos[bid*3+1], data_->xpos[bid*3+2]);
            const Eigen::Matrix<double,6,1> error =
                sim2realError(target->second, bid);
            const Eigen::Matrix<double,6,1> inverse_error =
                se3Log(R_target.transpose() * R_current,
                       R_target.transpose() * (p_current - target->second.position));
            const Eigen::Matrix<double,6,6> jlog =
                se3LeftJacobianInverse(-inverse_error);
            const Eigen::Matrix<double,6,Eigen::Dynamic> error_jacobian =
                -jlog * body_jacobian;

            Eigen::Matrix<double,6,1> weights;
            weights << entry.pos_weight, entry.pos_weight, entry.pos_weight,
                       entry.rot_weight, entry.rot_weight, entry.rot_weight;
            const Eigen::Matrix<double,6,1> weighted_error =
                weights.array() * (-error.array());
            const Eigen::Matrix<double,6,Eigen::Dynamic> weighted_jacobian =
                weights.asDiagonal() * error_jacobian;
            const double mu = weighted_error.squaredNorm();
            H += weighted_jacobian.transpose() * weighted_jacobian
                + mu * Eigen::MatrixXd::Identity(nv_, nv_);
            c -= weighted_jacobian.transpose() * weighted_error;
        }

        std::vector<double> lower(nv_, -1e30), upper(nv_, 1e30);
        for (const auto& bound : vel_bounds_) {
            const double q = data_->qpos[bound.qadr];
            lower[bound.vadr] = 0.95 * (bound.lo - q);
            upper[bound.vadr] = 0.95 * (bound.hi - q);
        }
        std::vector<double> H_flat(nv_ * nv_), c_flat(nv_);
        for (int col = 0; col < nv_; ++col)
            for (int row = 0; row < nv_; ++row)
                H_flat[row + col * nv_] = H(row,col);
        for (int i = 0; i < nv_; ++i) c_flat[i] = c[i];

        DAQPProblem qp;
        qp.n = nv_; qp.m = nv_; qp.ms = nv_;
        qp.H = H_flat.data(); qp.f = c_flat.data();
        qp.A = nullptr; qp.bupper = upper.data(); qp.blower = lower.data();
        qp.sense = nullptr; qp.nh = 0; qp.break_points = nullptr;
        std::vector<double> solution(nv_, 0.0), lambda(nv_, 0.0);
        DAQPResult result;
        result.x = solution.data(); result.lam = lambda.data();
        DAQPSettings settings;
        daqp_default_settings(&settings);
        daqp_quadprog(&result, &qp, &settings);
        if (result.exitflag <= 0)
            throw std::runtime_error("sim2real IK QP failed, DAQP exitflag="
                                     + std::to_string(result.exitflag));
        Eigen::Map<Eigen::VectorXd> delta(solution.data(), nv_);
        if (!delta.allFinite()) throw std::runtime_error("sim2real IK produced non-finite delta");
        mj_integratePos(model_, data_->qpos, delta.data(), 1.0);
        clampJoints();
        mj_forward(model_, data_);
    }

    // 计算当前任务组的总位姿误差，供收敛判断使用。
    double computeSim2realError(const BodyMap& targets,
                                const std::vector<IKEntry>& entries) const {
        double squared_norm = 0.0;
        for (const auto& entry : entries) {
            const auto target = targets.find(entry.human_body);
            if (target == targets.end() || entry.body_id < 0) continue;
            squared_norm += sim2realError(target->second, entry.body_id).squaredNorm();
        }
        return std::sqrt(squared_norm);
    }

    // 单阶段先求解一次，误差仍明显下降时最多追加十次迭代。
    void solveSim2realStage(const BodyMap& targets,
                            const std::vector<IKEntry>& entries) {
        if (entries.empty()) return;
        double current_error = computeSim2realError(targets, entries);
        runSim2realStep(targets, entries);
        double next_error = computeSim2realError(targets, entries);
        int extra_iterations = 0;
        while (current_error - next_error > 0.001 && extra_iterations < 10) {
            current_error = next_error;
            runSim2realStep(targets, entries);
            next_error = computeSim2realError(targets, entries);
            ++extra_iterations;
        }
    }

    void runIKStep(const BodyMap& targets, const std::vector<IKEntry>& entries)
    {
        const double dt = model_->opt.timestep * 10.0;
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(nv_, nv_) * damping_;
        Eigen::VectorXd f = Eigen::VectorXd::Zero(nv_);

        for (const auto& entry : entries) {
            auto tgt = targets.find(entry.human_body);
            if (tgt == targets.end()) continue;
            const int bid = entry.body_id;
            if (bid < 0) continue;

            Eigen::Matrix<double,3,Eigen::Dynamic,Eigen::RowMajor> jp_w(3,nv_), jr_w(3,nv_);
            mj_jacBody(model_, data_, jp_w.data(), jr_w.data(), bid);

            Eigen::Matrix3d R_wf;
            const mjtNum* xm = data_->xmat + bid*9;
            for (int r=0;r<3;r++) for(int c=0;c<3;c++) R_wf(r,c)=xm[r*3+c];

            Eigen::MatrixXd Jp = R_wf.transpose() * jp_w;
            Eigen::MatrixXd Jr = R_wf.transpose() * jr_w;
            if (entry.upper_body) {
                for (int v = 0; v < nv_; ++v) if (!upper_body_dofs_[v]) {
                    Jp.col(v).setZero();
                    Jr.col(v).setZero();
                }
            }

            Eigen::Vector3d cp(data_->xpos[bid*3],data_->xpos[bid*3+1],data_->xpos[bid*3+2]);
            Eigen::Vector3d pe = R_wf.transpose() * (tgt->second.position - cp);
            Eigen::Vector4d cq(data_->xquat[bid*4],data_->xquat[bid*4+1],
                               data_->xquat[bid*4+2],data_->xquat[bid*4+3]);
            Eigen::Vector3d re = quat::so3_error(cq, tgt->second.rot_wxyz);

            if (entry.pos_weight != 0) {
                Eigen::MatrixXd wJ = entry.pos_weight * Jp;
                Eigen::VectorXd we = entry.pos_weight * pe;
                H += wJ.transpose() * wJ;
                f -= wJ.transpose() * we;
                H += 1e-4 * Eigen::MatrixXd::Identity(nv_, nv_);
            }
            if (entry.rot_weight != 0) {
                Eigen::MatrixXd wJ = entry.rot_weight * Jr;
                Eigen::VectorXd we = entry.rot_weight * re;
                H += wJ.transpose() * wJ;
                f -= wJ.transpose() * we;
                H += 1e-4 * Eigen::MatrixXd::Identity(nv_, nv_);
            }
        }

        std::vector<double> vlo(nv_, -1e30), vhi(nv_, 1e30);
        for (auto& b : vel_bounds_) {
            double q = data_->qpos[b.qadr];
            vlo[b.vadr] = (b.lo - q) / dt;
            vhi[b.vadr] = (b.hi - q) / dt;
        }

        std::vector<double> H_flat(nv_*nv_), f_flat(nv_);
        for (int c=0;c<nv_;c++)
            for (int r=0;r<nv_;r++)
                H_flat[r + c*nv_] = H(r,c);
        for (int i=0;i<nv_;i++) f_flat[i] = f[i];

        DAQPProblem qp;
        qp.n=nv_; qp.m=nv_; qp.ms=nv_;
        qp.H=H_flat.data(); qp.f=f_flat.data();
        qp.A=nullptr; qp.bupper=vhi.data(); qp.blower=vlo.data();
        qp.sense=nullptr; qp.nh=0; qp.break_points=nullptr;

        DAQPResult res;
        std::vector<double> x_sol(nv_,0.0), lam_sol(nv_,0.0);
        res.x=x_sol.data(); res.lam=lam_sol.data();

        DAQPSettings settings;
        daqp_default_settings(&settings);
        daqp_quadprog(&res, &qp, &settings);

        Eigen::VectorXd v(nv_);
        if (res.exitflag > 0) {
            for (int i=0;i<nv_;i++) v[i]=res.x[i];
        } else {
            static auto last_daqp_fail = std::chrono::steady_clock::now() - std::chrono::seconds(10);
            auto now_daqp_fail = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now_daqp_fail - last_daqp_fail).count() >= 1.0) {
                last_daqp_fail = now_daqp_fail;
                std::printf("[WARN][GMR] daqp failed exitflag=%d, falling back to LDLT\n",
                            res.exitflag);
            }
            v = H.ldlt().solve(-f);
            for (auto& b : vel_bounds_)
                v[b.vadr] = std::clamp(v[b.vadr], vlo[b.vadr], vhi[b.vadr]);
        }

        if (!v.allFinite()) throw std::runtime_error("IK produced non-finite joint velocity");
        mj_integratePos(model_, data_->qpos, v.data(), dt);
        clampJoints();
        mj_forward(model_, data_);
    }

    void solveIK(const BodyMap& targets, const std::vector<IKEntry>& entries) {
        if (entries.empty()) return;
        double curr_error = computeError(targets, entries);
        runIKStep(targets, entries);
        double next_error = computeError(targets, entries);
        int num_iter = 0;
        while (curr_error - next_error > 0.001 && num_iter < 10) {
            curr_error = next_error;
            runIKStep(targets, entries);
            next_error = computeError(targets, entries);
            num_iter++;
        }
    }

    double computeError(const BodyMap& targets, const std::vector<IKEntry>& entries) const {
        double sq = 0.0;
        for (const auto& entry : entries) {
            auto it = targets.find(entry.human_body);
            if (it == targets.end()) continue;
            const int bid = entry.body_id;
            if (bid < 0) continue;
            Eigen::Vector3d cp(data_->xpos[bid*3],data_->xpos[bid*3+1],data_->xpos[bid*3+2]);
            Eigen::Vector4d cq(data_->xquat[bid*4],data_->xquat[bid*4+1],
                            data_->xquat[bid*4+2],data_->xquat[bid*4+3]);
            double pos_err = (it->second.position - cp).squaredNorm();
            double rot_err = quat::so3_error(cq, it->second.rot_wxyz).squaredNorm();
            sq += pos_err + rot_err;
        }
        return std::sqrt(sq);
    }
};

} // namespace gmr_mink
