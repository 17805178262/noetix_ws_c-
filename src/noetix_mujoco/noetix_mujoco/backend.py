"""MuJoCo physics backend for the Noetix LowCmd/LowState ROS 2 contract.

MuJoCo model/data are owned by the main physics thread. ROS callbacks only
write synchronized command/request snapshots. No hardware bridge is started.
"""
from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def _stamp(seconds):
    from builtin_interfaces.msg import Time
    sec = int(max(seconds, 0.0))
    nsec = int(round((max(seconds, 0.0)-sec)*1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    return Time(sec=sec,nanosec=nsec)


def _yaw_only_quat(quat):
    w,x,y,z = map(float,quat)
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return np.array([math.cos(yaw/2),0,0,math.sin(yaw/2)])


@dataclass(frozen=True)
class MotorTarget:
    q: float
    dq: float
    tau: float
    kp: float
    kd: float


class NoetixMujocoBackend:
    def __init__(self, robot_root: Path, config_dir: Path, config_override: str, headless: bool):
        import mujoco
        import rclpy
        from rclpy.node import Node
        from noetix_msgs.msg import LowCmd, LowState
        from std_msgs.msg import String, Bool
        from sensor_msgs.msg import JointState, Imu
        from rosgraph_msgs.msg import Clock
        from .model import build_model

        self.mj = mujoco
        self.spec = build_model(robot_root, config_dir, config_override)
        self.cfg = self.spec.config
        self.headless = headless or bool(self.cfg['headless'])
        self.model = mujoco.MjModel.from_xml_string(self.spec.mjcf)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0/self.cfg['physics_hz']
        self.node = Node('noetix_mujoco_backend')
        self.logger = self.node.get_logger()
        self.lock = threading.Lock()
        self.command = None
        self.command_rx_at = 0.0
        self.enabled = False
        self.hoist_request = None
        self.keyboard_hoist_active = False
        self.keyboard_hoist_rx = 0.0
        self._reset_requested = False
        self._paused = False
        self._last_key_at = {}
        self._manual_hoist = False
        self.running = True
        self._physics_error = None
        self._render_lock = threading.Lock()
        self._render_snapshot = None
        self.rx_total = 0
        self.rx_rejected = 0
        self.sequence = 0
        self.measured_rtf = 0.0
        self.measured_physics_hz = 0.0
        self.last_effort = np.zeros(len(self.spec.joints), dtype=np.float64)

        self.joint_qadr, self.joint_dadr, self.motor_id = [], [], []
        for spec in self.spec.joints:
            jid = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,spec.name)
            aid = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_ACTUATOR,spec.name+'_motor')
            if jid < 0 or aid < 0:
                raise ValueError('Model missing joint/motor: '+spec.name)
            if self.model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError('Expected hinge joint: '+spec.name)
            self.joint_qadr.append(int(self.model.jnt_qposadr[jid]))
            self.joint_dadr.append(int(self.model.jnt_dofadr[jid]))
            self.motor_id.append(aid)
        # Cached mappings and fixed scratch buffers for 1 kHz motor PD.
        # Avoid Python per-motor NumPy calls in the physics loop.
        self.motor_qadr = np.asarray(self.joint_qadr, dtype=np.intp)
        self.motor_dadr = np.asarray(self.joint_dadr, dtype=np.intp)
        self.motor_aid = np.asarray(self.motor_id, dtype=np.intp)
        self.motor_direction = np.asarray([s.direction for s in self.spec.joints], dtype=np.float64)
        self.motor_offset = np.asarray([s.offset for s in self.spec.joints], dtype=np.float64)
        self.motor_effort = np.asarray([s.effort for s in self.spec.joints], dtype=np.float64)
        n_motors = len(self.spec.joints)
        self._motor_q = np.empty(n_motors, dtype=np.float64)
        self._motor_dq = np.empty(n_motors, dtype=np.float64)
        self._motor_error = np.empty(n_motors, dtype=np.float64)
        self._motor_tau = np.empty(n_motors, dtype=np.float64)
        self._motor_scratch = np.empty(n_motors, dtype=np.float64)
        self.hoist_eq = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                           'noetix_hoist_weld')
        self.hoist_anchor = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,
                                               'noetix_hoist_anchor')
        self.mocap_id = int(self.model.body_mocapid[self.hoist_anchor])
        if self.hoist_eq < 0 or self.mocap_id < 0:
            raise ValueError('MuJoCo hoist constraint or anchor not found')
        self.sensor_ids = {}
        for key, name in (('quat','noetix_imu_quat'),('gyro','noetix_imu_gyro'),
                          ('acc','noetix_imu_accel')):
            sid = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_SENSOR,name)
            if sid < 0:
                raise ValueError('IMU sensor missing: '+name)
            adr = int(self.model.sensor_adr[sid])
            dim = int(self.model.sensor_dim[sid])
            self.sensor_ids[key] = slice(adr,adr+dim)
        self._reset_model()
        self._capture_render_snapshot()

        self.cmd_sub = self.node.create_subscription(LowCmd,'/noetix/low_cmd',self._on_cmd,20)
        self.request_sub = self.node.create_subscription(String,'/noetix/control_request',
                                                          self._on_control,20)
        self.keyboard_sub = self.node.create_subscription(Bool,'/noetix/sim/keyboard_hoist',
                                                           self._on_keyboard_hoist,1)
        self.state_pub = self.node.create_publisher(LowState,'/noetix/low_state',10)
        self.joint_pub = self.node.create_publisher(JointState,'/joint_states',10)
        self.imu_pub = self.node.create_publisher(Imu,'/noetix/imu',10)
        self.clock_pub = self.node.create_publisher(Clock,'/clock',10)
        self.control_state_pub = self.node.create_publisher(String,'/noetix/control_state',10)
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.hoist_pub = self.node.create_publisher(String,'/noetix/sim/hoist_state',qos)
        self._hoist_active = False
        self._publish_hoist()
        self.logger.info(f'robot={self.spec.robot_name} dof={len(self.spec.joints)} '
                         f'physics={self.cfg["physics_hz"]}Hz state={self.cfg["state_publish_hz"]}Hz '
                         f'actuator={self.cfg["actuator_mode"]} drive=DISABLED')

    def _reset_model(self):
        self.mj.mj_resetData(self.model,self.data)
        for qadr, item in zip(self.joint_qadr,self.spec.joints):
            self.data.qpos[qadr] = item.initial_q
        self.data.qpos[0:3] = [0,0,self.spec.spawn_z]
        self.data.qpos[3:7] = [1,0,0,0]
        self.data.qvel[:] = 0
        self.data.ctrl[:] = 0
        self.data.eq_active[self.hoist_eq] = 0
        self.data.mocap_pos[self.mocap_id] = [0,0,self.cfg['hoist_base_z']]
        self.data.mocap_quat[self.mocap_id] = [1,0,0,0]
        self.last_effort[:] = 0
        self.mj.mj_forward(self.model,self.data)

    def _on_cmd(self, msg):
        self.rx_total += 1
        if len(msg.motor_cmd) != len(self.spec.joints):
            self.rx_rejected += 1
            return
        commands = {}
        for item in msg.motor_cmd:
            if item.name in commands or not all(math.isfinite(float(x)) for x in
                 (item.q,item.dq,item.tau,item.kp,item.kd)) or item.kp < 0 or item.kd < 0:
                self.rx_rejected += 1
                return
            commands[item.name] = MotorTarget(float(item.q),float(item.dq),float(item.tau),
                                               float(item.kp),float(item.kd))
        if set(commands) != {s.actuator for s in self.spec.joints}:
            self.rx_rejected += 1
            return
        ordered = tuple(commands[s.actuator] for s in self.spec.joints)
        # Validate once per received LowCmd (typically 50 Hz), rather than
        # unpacking 29 Python MotorTarget objects at every 1 kHz physics step.
        prepared = tuple(np.asarray(values, dtype=np.float64) for values in (
            [c.q for c in ordered],
            [c.dq for c in ordered],
            [c.tau for c in ordered],
            [c.kp for c in ordered],
            [c.kd for c in ordered],
        ))
        with self.lock:
            if self.enabled:
                self.command = prepared
                self.command_rx_at = time.monotonic()
            else:
                self.rx_rejected += 1

    def _on_control(self,msg):
        value = msg.data.strip().lower()
        with self.lock:
            if value == 'enable':
                if not self.enabled:
                    self.enabled = True
                    self.command = None
                    self.logger.info('SIM drive -> ENABLED')
            elif value in ('disable','stop','estop','safe_stop'):
                self.enabled = False
                self.command = None
                self.logger.info('SIM drive -> DISABLED')
            # stand/lie intentionally consumed by the shared pose-mode node.

    def _on_keyboard_hoist(self,msg):
        with self.lock:
            self.keyboard_hoist_active = bool(msg.data)
            self.keyboard_hoist_rx = time.monotonic()

    def _on_viewer_action(self, action):
        """GLFW main thread sends requests; physics thread applies them."""
        with self.lock:
            if action == 'hoist_toggle':
                self._manual_hoist = not self._manual_hoist
            elif action == 'reset':
                self._reset_requested = True
            elif action == 'pause_toggle':
                self._paused = not self._paused

    def _publish_hoist(self):
        from std_msgs.msg import String
        self.hoist_pub.publish(String(data='hold' if self._hoist_active else 'released'))

    def _set_hoist(self, active):
        if active == self._hoist_active:
            return
        if active:
            self.data.qpos[2] = self.cfg['hoist_base_z']
            self.data.qpos[3:7] = _yaw_only_quat(self.data.qpos[3:7])
            self.data.qvel[:6] = 0
            self.data.qacc_warmstart[:] = 0
            self.data.mocap_pos[self.mocap_id] = self.data.qpos[:3]
            self.data.mocap_quat[self.mocap_id] = self.data.qpos[3:7]
            self.data.eq_active[self.hoist_eq] = 1
        else:
            self.data.eq_active[self.hoist_eq] = 0
        self._hoist_active = active
        self.mj.mj_forward(self.model,self.data)
        self._publish_hoist()
        self.logger.info('Hoist '+('HOLD' if active else 'RELEASED'))

    def _process_requests(self):
        with self.lock:
            reset = self._reset_requested
            self._reset_requested = False
            if reset:
                self.enabled = False
                self.command = None
                self.keyboard_hoist_active = False
                self._manual_hoist = False
            external_hold = (self.keyboard_hoist_active and
                             time.monotonic() - self.keyboard_hoist_rx < 0.35)
            desired_hoist = self._manual_hoist or external_hold
        if reset:
            self._reset_model()
            self._hoist_active = False
            self._publish_hoist()
            self.logger.info('Simulation reset, drive disabled')
        if desired_hoist != self._hoist_active:
            self._set_hoist(bool(desired_hoist))

    def _capture_render_snapshot(self):
        """Publish a small immutable snapshot; never expose live physics MjData."""
        sample = (
            self.data.qpos.copy(),
            self.data.qvel.copy(),
            self.data.mocap_pos.copy(),
            self.data.mocap_quat.copy(),
            self.data.eq_active.copy(),
            float(self.data.time),
        )
        with self._render_lock:
            self._render_snapshot = sample

    def _get_render_snapshot(self):
        # Snapshot arrays are never modified after publication.
        with self._render_lock:
            return self._render_snapshot

    def _motor_step(self):
        with self.lock:
            enabled = self.enabled
            fresh = self.command is not None and (
                time.monotonic()-self.command_rx_at <= self.cfg['command_timeout_s'])
            command = self.command if enabled and fresh else None
        if command is None:
            self.data.ctrl[:] = 0
            self.last_effort[:] = 0
        else:
            target_q, target_dq, feedforward_tau, kp, kd = command
            # All vectors are in the actuator / ROS LowCmd convention.
            np.take(self.data.qpos, self.motor_qadr, out=self._motor_q)
            np.subtract(self._motor_q, self.motor_offset, out=self._motor_q)
            np.multiply(self._motor_q, self.motor_direction, out=self._motor_q)
            np.take(self.data.qvel, self.motor_dadr, out=self._motor_dq)
            np.multiply(self._motor_dq, self.motor_direction, out=self._motor_dq)

            # tau = clip(kp*(q_des-q[-dt*dq]) + kd*(dq_des-dq) + tau_ff, effort)
            np.subtract(target_q, self._motor_q, out=self._motor_error)
            if self.cfg['actuator_mode'] == 'predictive_pd':
                np.multiply(self._motor_dq, self.model.opt.timestep,
                            out=self._motor_scratch)
                np.subtract(self._motor_error, self._motor_scratch,
                            out=self._motor_error)
            np.multiply(kp, self._motor_error, out=self._motor_tau)
            np.subtract(target_dq, self._motor_dq, out=self._motor_scratch)
            np.multiply(kd, self._motor_scratch, out=self._motor_scratch)
            np.add(self._motor_tau, self._motor_scratch, out=self._motor_tau)
            np.add(self._motor_tau, feedforward_tau, out=self._motor_tau)
            np.clip(self._motor_tau, -self.motor_effort, self.motor_effort,
                    out=self._motor_tau)
            np.multiply(self._motor_tau, self.motor_direction, out=self._motor_tau)
            self.data.ctrl[self.motor_aid] = self._motor_tau
            self.last_effort[:] = self._motor_tau

        damping = self.cfg['joint_viscous_damping']
        coulomb = self.cfg['joint_coulomb_friction']
        self.data.qfrc_applied[:] = 0
        if damping or coulomb:
            for dadr in self.joint_dadr:
                v = float(self.data.qvel[dadr])
                friction = math.copysign(coulomb, v) if abs(v) > 1e-5 else 0.0
                self.data.qfrc_applied[dadr] = -damping*v-friction

    def _publish_state(self):
        from noetix_msgs.msg import LowState, MotorState, RobotStatus
        from sensor_msgs.msg import JointState, Imu
        stamp = _stamp(self.data.time)
        self.sequence += 1
        state = LowState()
        state.stamp = stamp
        state.sequence = self.sequence
        joints = JointState()
        joints.header.stamp = stamp
        for index,(spec,qadr,dadr) in enumerate(zip(self.spec.joints,self.joint_qadr,self.joint_dadr)):
            q = float(self.data.qpos[qadr])
            dq = float(self.data.qvel[dadr])
            m = MotorState()
            m.name = spec.actuator
            m.q = float(spec.direction*(q-spec.offset))
            m.dq = float(spec.direction*dq)
            m.tau_est = float(spec.direction*self.last_effort[index])
            m.temperature = 0.0
            m.error = 0
            state.motor_state.append(m)
            joints.name.append(spec.name)
            joints.position.append(q)
            joints.velocity.append(dq)
            joints.effort.append(float(self.last_effort[index]))
        quat = self.data.sensordata[self.sensor_ids['quat']]
        gyro = self.data.sensordata[self.sensor_ids['gyro']]
        acc = self.data.sensordata[self.sensor_ids['acc']]
        state.imu.valid = True
        state.imu.quaternion = [float(x) for x in quat]
        state.imu.gyroscope = [float(x) for x in gyro]
        state.imu.accelerometer = [float(x) for x in acc]
        with self.lock:
            enabled = self.enabled
        state.status.mode = RobotStatus.MODE_ENABLED if enabled else RobotStatus.MODE_DISABLED
        state.status.connected = True
        state.status.enabled = enabled
        state.status.fault = 0
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.spec.imu_link
        imu.orientation.w,imu.orientation.x,imu.orientation.y,imu.orientation.z = map(float,quat)
        imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z = map(float,gyro)
        imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z = map(float,acc)
        self.state_pub.publish(state)
        self.joint_pub.publish(joints)
        self.imu_pub.publish(imu)

    def _publish_clock(self):
        from rosgraph_msgs.msg import Clock
        self.clock_pub.publish(Clock(clock=_stamp(self.data.time)))

    def _publish_stats(self):
        from std_msgs.msg import String
        with self.lock:
            mode = 'ENABLED' if self.enabled else 'DISABLED'
            fresh = self.command is not None and (
                time.monotonic()-self.command_rx_at <= self.cfg['command_timeout_s'])
        self.control_state_pub.publish(String(
            data=f'mode={mode} command={"present" if fresh else "none"} '
                 f'sim_time={self.data.time:.3f} hoist={"HOLD" if self._hoist_active else "RELEASED"} '
                 f'cmd_rx={self.rx_total} cmd_reject={self.rx_rejected} '
                 f'rtf={self.measured_rtf:.2f} physics_actual_hz={self.measured_physics_hz:.0f}'))

    def _physics_loop(self):
        """Single owner of live MjData, including hoist, actuators and ROS state."""
        dt = float(self.model.opt.timestep)
        control_steps = round(self.cfg['physics_hz']/self.cfg['control_frequency'])
        state_steps = round(self.cfg['physics_hz']/self.cfg['state_publish_hz'])
        clock_steps = round(self.cfg['physics_hz']/self.cfg['clock_publish_hz'])
        render_steps = max(1, round(self.cfg['physics_hz']/self.cfg['gui_fps']))
        deadline = time.monotonic()
        step = 0
        speed_wall = time.monotonic()
        speed_sim = float(self.data.time)
        speed_steps = 0
        while self.running:
            self._process_requests()
            with self.lock:
                paused = self._paused
            if not paused:
                if step % control_steps == 0:
                    self._motor_step()
                self.mj.mj_step(self.model, self.data)
                step += 1
                if step % clock_steps == 0:
                    self._publish_clock()
                if step % state_steps == 0:
                    self._publish_state()
                if not self.headless and step % render_steps == 0:
                    self._capture_render_snapshot()
            now = time.monotonic()
            if now - speed_wall >= 2.0:
                wall_delta = now - speed_wall
                self.measured_rtf = (float(self.data.time)-speed_sim) / wall_delta
                self.measured_physics_hz = (step-speed_steps) / wall_delta
                self._publish_stats()
                if not paused:
                    self.logger.info(
                        f'MuJoCo speed: RTF={self.measured_rtf:.2f}x '
                        f'actual_physics={self.measured_physics_hz:.0f}Hz '
                        f'target={self.cfg["physics_hz"]:.0f}Hz '
                        f'GUI={"off" if self.headless else "lightweight"}')
                speed_wall, speed_sim, speed_steps = now, float(self.data.time), step
            if self.cfg['realtime'] and not paused:
                deadline += dt
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                elif remaining < -0.2:
                    deadline = time.monotonic()
            elif paused:
                deadline = now
                time.sleep(0.01)

    def _physics_guard(self):
        try:
            self._physics_loop()
        except BaseException as exc:
            self._physics_error = exc
            self.running = False
            self.logger.error(f'Physics loop failed: {exc!r}')

    def run(self):
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        ros_thread = threading.Thread(target=executor.spin, name='mujoco_ros', daemon=True)
        ros_thread.start()
        viewer = None
        physics_thread = None
        try:
            if self.headless:
                # No GLFW and no OpenGL context in server/headless mode.
                self._physics_loop()
            else:
                # GLFW/OpenGL stays on the main thread; physics owns its own
                # MjData on a separate thread. Renderer owns a second MjData.
                from .lightweight_viewer import LightweightViewer
                viewer = LightweightViewer(
                    self.mj, self.model, get_snapshot=self._get_render_snapshot,
                    on_action=self._on_viewer_action, config=self.cfg,
                    robot_name=self.spec.robot_name, spawn_z=self.spec.spawn_z)
                viewer.open()
                self.logger.info(
                    'Lightweight GLFW viewer ready: H hoist toggle, R reset, '
                    'Space pause, mouse rotate/pan/zoom')
                physics_thread = threading.Thread(
                    target=self._physics_guard, name='mujoco_physics', daemon=True)
                physics_thread.start()
                viewer.run(lambda: self.running)
        finally:
            self.running = False
            if physics_thread is not None:
                physics_thread.join(timeout=5.0)
            if viewer is not None:
                viewer.close()
            executor.shutdown()
            ros_thread.join(timeout=2.0)
            self.node.destroy_node()
        if self._physics_error is not None:
            raise RuntimeError('MuJoCo physics loop failed') from self._physics_error


def main():
    parser = argparse.ArgumentParser(description='MuJoCo Noetix ROS 2 simulation')
    parser.add_argument('--robot-root',required=True)
    parser.add_argument('--config-dir',required=True)
    parser.add_argument('--sim-config',default='')
    parser.add_argument('--headless',action='store_true')
    args, ros_args = parser.parse_known_args()
    import rclpy
    rclpy.init(args=ros_args)
    try:
        backend = NoetixMujocoBackend(Path(args.robot_root),Path(args.config_dir),
                                       args.sim_config,args.headless)
        backend.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
