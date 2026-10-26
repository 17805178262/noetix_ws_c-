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
        ground_height_   = j["ground_height"].get<double>();
        foot_target_height_ = j.value("foot_target_height", 0.06);
        robot_ground_height_ = j.value("robot_ground_height", 0.0);
        align_robot_sole_ = j.value("align_robot_sole", false);
        lock_planted_feet_ = j.value("lock_planted_feet", false);
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
        // 上半身独立求解，手臂目标误差和收敛次数都不能反过来改变根节点或腿部。
        auto split_upper = [](std::vector<IKEntry>& entries, std::vector<IKEntry>& upper) {
            for (const auto& entry : entries) if (entry.upper_body) upper.push_back(entry);
            entries.erase(std::remove_if(entries.begin(), entries.end(),
                [](const IKEntry& entry) { return entry.upper_body; }), entries.end());
        };
        split_upper(entries1_, upper_entries1_);
        split_upper(entries2_, upper_entries2_);

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
        BodyMap scaled = scaleHumanData(human_data);
        auto targets = [&](const auto& positions, const auto& rotations, auto& contacts) {
            BodyMap result = applyGroundOffset(offsetHumanData(scaled, positions, rotations));
            if (offset_to_ground) result = offsetToGround(result);
            if (lock_planted_feet_) stabilizePlantedFeet(result, frame_time, contacts);
            return result;
        };
        BodyMap offset = targets(pos_offsets1_, rot_offsets1_, foot_contacts1_);
        // 两阶段使用各自的目标偏移；此前第二阶段实际上沿用了第一阶段的偏移。
        BodyMap offset2;
        if (use_table2_) offset2 = targets(pos_offsets2_, rot_offsets2_, foot_contacts2_);
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
        if (offset_to_ground && align_robot_sole_) alignRobotFeetToGround();

        Eigen::VectorXd out(nq_);
        for (int i = 0; i < nq_; ++i) out[i] = data_->qpos[i];
        return out;
    }

    const mjModel* model() const { return model_; }

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
    bool lock_planted_feet_ = false;
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

    BodyMap offsetToGround(BodyMap src) const {
        double lowest = std::numeric_limits<double>::infinity();
        for (auto& [name,bd] : src) {
            std::string lo = name;
            std::transform(lo.begin(),lo.end(),lo.begin(),::tolower);
            // Different mocap formats name the lowest tracked point
            // differently: XRobot uses Foot, while FBX commonly uses ToeBase.
            if (lo.find("foot") != std::string::npos ||
                lo.find("toe") != std::string::npos ||
                lo.find("ankle") != std::string::npos)
                lowest = std::min(lowest, bd.position[2]);
        }
        if (!std::isinf(lowest))
            for (auto& [name,bd] : src)
                bd.position[2] = bd.position[2] - lowest + foot_target_height_;
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

    void alignRobotFeetToGround() {
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
            data_->qpos[2] += robot_ground_height_ - sole_z;
            mj_forward(model_, data_);
        }
    }

    void clampJoints() {
        for (auto& b : vel_bounds_)
            data_->qpos[b.qadr] = std::clamp(data_->qpos[b.qadr], b.lo, b.hi);
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
