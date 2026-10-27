# Noetix C++ 部署启动说明

本工作区的 `noetix_policy` 是独立 C++ 策略包，直接编译、直接启动。
真机、Gazebo 和 MuJoCo 共用 LowCmd/LowState 接口。每个新终端都要加载 ROS 和当前工作区环境。
参数使用 `参数名:=值`，布尔值使用 `true` / `false`。
`robot_name` 选择底层机器人资源，不会自动替换策略模型。

## 1. 编译与环境

```bash
cd /home/byf/scy/noetix_ws_c++
source /opt/ros/humble/setup.bash
./build.sh
source install/setup.bash
```

`build.sh` 编译当前C++工程，保留真机、GMR、策略和MuJoCo，明确排除Gazebo；
板子不需要安装Gazebo。编译产物位于 `build/`、`install/`、`log/`，不生成跨架构安装包。
可用 `./build.sh --parallel-workers 2` 限制同时构建的包数。

根目录的 `start.sh` 自动加载本工作区的 `install/setup.bash`：

```bash
# 真机：同时启动底层、姿态节点、手柄及C++策略切换器。
./start.sh real robot_name:=n3 use_rviz:=false
# 真机并启动Pico/GMR；visualize打开MuJoCo参考查看器。
./start.sh real robot_name:=n3 use_rviz:=false \
  device:=pico human_height:=1.78 publish_hz:=50.0 visualize:=true
# MuJoCo物理仿真并启动策略；不启动真机硬件或Gazebo。
./start.sh sim robot_name:=n3
# 无显示窗口的MuJoCo仿真。
./start.sh sim robot_name:=n3 gui:=false
```

省略模式时默认 `real`，例如 `./start.sh robot_name:=n3`。
真机模式将 `joystick_config:=...` 传给策略，其余参数传给 `real.launch.py`；
仿真模式将参数传给MuJoCo的 `sim.launch.py`，默认 `launch_policy:=true`。
两种模式选择一种运行。脚本不会自动使能或激活策略，仍通过手柄操作。
真机模式中任意一个launch退出时会停止另一个；按Ctrl+C退出本脚本启动的进程。
`./build.sh --help`、`./start.sh --help` 可查看用法。

只部署真机、不需要仿真后端时：

```bash
colcon build --packages-skip noetix_gazebo noetix_mujoco --cmake-args -DCMAKE_BUILD_TYPE=Release
```

已有完整安装后，只更新策略或机器人资源时，按需执行：

```bash
colcon build --packages-select noetix_policy
colcon build --packages-select noetix_robots
source install/setup.bash
```

更新模型、配置或源码后需要重新编译，并重启对应进程。
策略使用 C++17、Eigen3、yaml-cpp、OpenSSL、minizip 和 ONNX Runtime C/C++ SDK。
本项目通过 `/opt/onnxruntime` 下的官方 CMake Config 查找 ONNX Runtime。
首次缺少 SDK 时可运行安装脚本；它支持 x86_64/aarch64，并会检查库与头文件：

```bash
bash src/noetix_policy/scripts/install_onnxruntime_official.sh
# 离线使用已有官方 SDK 压缩包：
# bash src/noetix_policy/scripts/install_onnxruntime_official.sh --archive /绝对路径/sdk.tgz
```

替换旧 SDK 后首次构建加 `--cmake-clean-cache`，以后正常 `colcon build`。
MuJoCo 仿真另外需要 ROS 使用的 Python 解释器能导入 MuJoCo；缺少依赖时运行：

```bash
bash src/noetix_mujoco/scripts/install_mujoco.sh
```

以上安装脚本按需使用，不在普通编译或启动过程中自动执行。

## 2. 真机启动：real.launch.py

终端一启动底层、手柄和 RViz：

```bash
ros2 launch noetix_bringup real.launch.py robot_name:=n3
```

终端二启动策略节点：

```bash
ros2 launch noetix_policy policy.launch.py
```

`real.launch.py` 不自动启动 policy 节点，只启动底层、姿态执行节点和手柄驱动。
另一个终端启动 policy 后，公共遥控器配置统一处理使能并进入默认姿态、失能、站立及策略切换。

| 参数 | 默认值 | 含义和作用 |
| --- | --- | --- |
| `robot_name` | `n3` | 选择机器人模型、硬件配置和标定 |
| `control_frequency_hz` | 空，实际为 `1000` | 底层收发和控制循环频率，单位 Hz；与策略推理频率独立 |
| `use_gamepad` | `true` | 启动手柄驱动；模式和策略按键由单独启动的 policy 处理 |
| `joy_device_id` | `0` | 手柄设备编号，多手柄时选择对应设备 |
| `joy_deadzone` | `0.001` | 原始手柄轴死区；策略指令处理由对应策略代码决定 |
| `use_rviz` | `true` | 有可用配置时打开 RViz |
| `rt_realtime` | `true` | 为硬件控制线程申请 SCHED_FIFO 实时调度 |
| `rt_lock_memory` | `true` | 锁定进程内存，减少换页造成的控制延迟 |
| `rt_strict` | `false` | `true` 时实时调度或锁内存失败就终止启动；`false` 时告警后继续 |
| `rt_priority` | `80` | 实时线程优先级 |
| `rt_cpu` | `-1` | 控制线程绑定的 CPU 核；`-1` 表示不指定 |

关闭 RViz 的示例：

```bash
ros2 launch noetix_bringup real.launch.py robot_name:=bumi2 use_rviz:=false
```

## 3. Gazebo 启动：sim.launch.py

同时启动 Gazebo 和策略节点：

```bash
ros2 launch noetix_gazebo sim.launch.py robot_name:=n3 launch_policy:=true
```

只启动仿真、不启动策略节点：

```bash
ros2 launch noetix_gazebo sim.launch.py robot_name:=bumi2
```

| 参数 | 默认值 | 含义和作用 |
| --- | --- | --- |
| `robot_name` | `n3` | 选择机器人资源 |
| `gui` | `true` | 打开 Gazebo 图形窗口；`false` 时只运行仿真服务 |
| `launch_policy` | `false` | 是否一起启动策略节点；节点加载后仍需按键激活策略 |
| `use_gamepad` | `true` | 启动手柄驱动；按键由 policy 的公共配置处理 |
| `joy_device_id` | `0` | 手柄设备编号 |
| `joy_deadzone` | `0.001` | 原始手柄轴死区 |
| `sim_config` | 空 | 仿真 YAML 覆盖文件的绝对路径；不填时读取包内公共配置与机型配置 |
| `joystick_config` | 策略包 `config/joystick.yaml` | 联合启动策略时使用的公共遥控器配置 |

Gazebo 窗口中按住 **Ctrl+R 或 Ctrl+Shift+R** 吊起机器人，松开后释放悬吊；
窗口失焦也会释放。此操作保持当前关节目标和策略，不重新初始化机器人。
仿真的物理频率、底层控制频率、状态发布频率和接触参数通过 `sim_config`
指定的 YAML 覆盖，不是直接的 launch 参数。

### 独立 MuJoCo 仿真后端

MuJoCo 物理仿真与第5节的GMR参考查看器是两个不同入口。
启动物理仿真和策略切换器：

```bash
ros2 launch noetix_mujoco sim.launch.py robot_name:=n3 gui:=true launch_policy:=true
```

不打开窗口或使用其他性能预设：

```bash
ros2 launch noetix_mujoco sim.launch.py robot_name:=n3 gui:=false launch_policy:=true
ros2 launch noetix_mujoco sim.launch.py robot_name:=n3 launch_policy:=true \
  sim_config:="$(ros2 pkg prefix noetix_mujoco)/share/noetix_mujoco/config/low_overhead.yaml"
```

MuJoCo入口同样支持 `robot_name`、`gui`、`launch_policy`、`use_gamepad`、
`joy_device_id`、`joy_deadzone`、`joystick_config`、`sim_config` 和第5节的联合动捕参数。
默认 `robot_name=n3`、`gui=true`、`launch_policy=false`；默认使用包内 `config/default.yaml`。

| 预设文件 | 物理/电机频率 | 状态发布 | 时钟发布 | GUI上限 |
| --- | --- | --- | --- | --- |
| `default.yaml` / `balanced.yaml` | 500 Hz | 250 Hz | 100 Hz | 50 FPS |
| `low_overhead.yaml` | 400 Hz | 200 Hz | 100 Hz | 20 FPS |
| `high_fidelity.yaml` | 1000 Hz | 500 Hz | 200 Hz | 60 FPS |

这些是仿真后端频率，不是策略推理频率。日志 `MuJoCo speed` 中 RTF 接近1表示接近实时运行。
MuJoCo窗口按 **H** 切换悬吊、**R** 重置并失能、**空格** 暂停/恢复；此处 Ctrl+R 不执行悬吊。
真机、Gazebo和MuJoCo后端每次选择一个运行，避免在相同话题上同时发送反馈和接收控制。

## 4. 遥控器切换与独立策略

policy 统一读取公共遥控器配置，发起模式请求并切换独立策略：

```bash
# 真机先启动底层，再运行policy；按7使能并进入默认姿态，按2进入默认站立。
ros2 launch noetix_policy policy.launch.py
# 连接Gazebo或MuJoCo时使用仿真时钟。
ros2 launch noetix_policy policy.launch.py use_sim_time:=true
# Gazebo联合启动会自动设置仿真时钟。
ros2 launch noetix_gazebo sim.launch.py robot_name:=n3 launch_policy:=true
```

| 参数 | 默认值 | 含义和作用 |
| --- | --- | --- |
| `joystick_config` | 包内 `config/joystick.yaml` | 手柄话题、轴、模式动作、策略名称和模型/motion路径 |
| `use_sim_time` | `false` | 使用仿真的 `/clock`；同时传给启动的独立策略节点 |

同一个遥控器配置分为 `controls`（机器人模式）和 `policies`（独立策略）两段：

| 按钮 | 动作 |
| --- | --- |
| 7 | 使能并平滑进入默认姿态：`action: enable` |
| 6 | 阻尼3秒后失能：`action: disable` |
| 6+7 | 立即失能：`action: estop` |
| 3 | 人工电机清错：`action: clear_errors` |
| 2 | 默认站立：`action: stand` |
| 0 | 行走：`policy: walk` |
| 4 | GMT：`policy: gmt` |
| 4+5 | motion：`policy: motion` |

使能请求会先停止当前策略，等底层反馈已使能后，再从实测关节位置平滑过渡到默认姿态。
N3和bumi2在 `robot.yaml` 的 `enable_control`、`enable_joint_angles` 中配置使能姿态，
肩第二关节为左 `+5°`、右 `-5°`，其余关节目标保持原配置。
未配置使能姿态的机型默认所有逻辑关节目标为0，增益为kp=6、kd=2，过渡2秒、发布100Hz。
原独立lie模式已移除，3号键用于人工电机清错；站立仍由2号键单独请求。
普通失能先由底层保持 `kp=0、kd=8` 的阻尼指令，3秒后真正关闭驱动。
阻尼期间状态为 `MODE_DAMPING`，`enabled=true`；旧策略命令被拒绝，重复失能不会延长计时。
重新使能会取消阻尼计时；`estop`、`stop`、`safe_stop` 始终立即失能，6+7为默认急停组合键。
失能和急停不会自动清错。完全失能后按3号键才调用SDK清错，不改变标定、不自动使能。
Bumi系列调用厂商 `CleanAllActuaorError()`；当前N3 SDK没有提供此接口，默认不支持。
清错日志表示请求已发送，是否实际解除故障还需根据电机反馈确认；仿真没有硬件电机错误。

`controls` 里只写 `action`，`policies` 里只写 `policy`，不再混放于 `bindings`。
两段都支持单键 `button` 和组合键 `buttons`；键位不能跨段重复，修改后重启policy即可。
指定源码中的公共配置示例：

```bash
ros2 launch noetix_policy policy.launch.py \
  joystick_config:=/home/byf/scy/noetix_ws_c++/src/noetix_policy/config/joystick.yaml
```

`policy` 填写 `src/noetix_policy/src/registry.cpp` 中的注册名称；旧的 `noetix_policy.policies.walk/gmt/motion` 写法仍可使用。

也可以直接运行一个示例，省略切换器；以下是使用仿真时钟的示例，真机运行时省略末尾参数：

```bash
ros2 run noetix_policy walk_policy --ros-args -p use_sim_time:=true
ros2 run noetix_policy gmt_policy --ros-args -p use_sim_time:=true
ros2 run noetix_policy motion_policy --ros-args -p use_sim_time:=true
```

walk和GMT使用N3模型，motion使用bumi4340模型及现有50fps动作文件。
选择其他底层机型时，需提供匹配的策略模型和观测定义。

模型与轨迹直接写在 `joystick.yaml` 的对应策略条目下，不需要其他YAML。
下面是 `joystick.policies` 的示例，话题、轴和 `controls` 保留原配置：

```yaml
policies:
  - button: 0
    policy: walk
    model: package://noetix_policy/models/walk.onnx
  - button: 4
    policy: gmt
    model: /home/nano/models/gmt_206000.onnx
  - buttons: [4, 5]
    policy: motion
    model: package://noetix_policy/models/motion.onnx
    motion: /home/nano/motions/dance.npz
```

每种策略只保留一个条目。`model` 和 `motion` 支持 `package://`、绝对路径，
以及相对于 `joystick.yaml` 所在目录的路径。使用外部配置时，先复制完整默认文件：

```bash
mkdir -p /home/nano/deploy
cp "$(ros2 pkg prefix noetix_policy)/share/noetix_policy/config/joystick.yaml" /home/nano/deploy/joystick.yaml

# 修改该文件中的资源路径后，退出策略、再次按键进入即可，无需再编译。
ros2 launch noetix_policy policy.launch.py joystick_config:=/home/nano/deploy/joystick.yaml
# MuJoCo联合启动同样支持。
ros2 launch noetix_mujoco sim.launch.py robot_name:=n3 launch_policy:=true \
  joystick_config:=/home/nano/deploy/joystick.yaml
# 独立策略节点也使用同一个配置参数。
ros2 run noetix_policy motion_policy --ros-args -p joystick_config:=/home/nano/deploy/joystick.yaml
```

如果直接编辑源码中的配置，需要启动时把 `joystick_config` 指向该源码文件；
默认读取的是 `install` 中的副本，普通构建不会把两者实时关联。
模型和轨迹路径在每次进入策略时重新读取，独立运行的策略需要重启节点；
按键绑定在切换器启动时读取，修改按键需要重启切换器。运行中的策略不会自动切换文件。
同一路径的ONNX文件大小或修改时间变化后，下次进入会重新加载、校验和预热；
未变化的模型继续复用会话，更换路径后不继续在缓存中保留该策略的旧模型。
替换文件时不要保留旧文件的修改时间，建议使用新文件名。
新模型加载或预热可能使切换稍慢，加载失败时保留原策略；加载前的原策略继续运行。
日志显示实际模型路径、SHA256和motion路径。输入输出名称、维度会校验，
但观测顺序、单位、缩放和动作定义仍必须与训练一致；motion仍需符合现有50fps/关节字段要求。

部署实现集中在 `src/noetix_policy/src/policies/walk.cpp`、`gmt.cpp`、`motion.cpp`，
各文件包含对应训练参数和观测入口；公共数学及历史缓冲保留在 `core.cpp`。
训练定义改变时，修改相应C++策略并编译；新增策略时，声明其契约/策略类，
在 `registry.cpp` 增加注册条目并把新文件加入 `CMakeLists.txt`，然后在手柄配置中填写注册名称。
资源配置与注册表只在初始化、切换时使用，50Hz推理循环不读取配置或文件。

三个策略的目标周期为20ms，即50Hz；周期由策略源码设置。
底层 `control_frequency_hz`、GMR `publish_hz` 和策略推理频率分别设置。

GMT默认使用CPU和2个ONNX计算线程，可在启动前明确设置：

```bash
export NOETIX_GMT_BACKEND=cpu
export NOETIX_GMT_ORT_THREADS=2
ros2 launch noetix_policy policy.launch.py
```

线程选项支持 `1`、`2`、`4`、`auto`。`cuda` 后端需要匹配的CUDA版ORT，
上述安装脚本安装的是CPU版SDK。模型会话会缓存，更改环境变量后需重启策略进程。
启动日志可核对模型指纹和线程数；`GMT timing` 显示实际发布频率、周期间隔、
观测/推理耗时及反馈年龄。配置50Hz与实际达到50Hz需要分别确认。

## 5. GMR 动捕：联合启动和独立启动

真机底层与 GMR 一起启动，另一个终端按第 2 节启动策略：

```bash
ros2 launch noetix_bringup real.launch.py robot_name:=n3 \
  device:=pico human_height:=1.78 publish_hz:=50.0 visualize:=true
```

Gazebo、策略和 GMR 一起启动：

```bash
ros2 launch noetix_gazebo sim.launch.py robot_name:=n3 \
  launch_policy:=true device:=pico human_height:=1.78 \
  publish_hz:=50.0 visualize:=true
```

只启动 GMR，独立测试动捕和 MuJoCo 显示：

```bash
ros2 launch noetix_gmr retarget.launch.py robot:=n3 device:=pico \
  human_height:=1.78 publish_hz:=30.0 visualize:=true
```

联合启动时，GMR 的开关规则如下：

| 参数或写法 | 作用 |
| --- | --- |
| `gmr:=auto`（默认） | 不传 GMR 参数时关闭；传入任意非空 GMR 参数时开启，例如 `device:=pico` |
| `gmr:=true` | 使用 GMR 默认参数启动；设备默认 Pico |
| `gmr:=false` | 强制关闭 GMR，即使还填写了设备等参数 |
| `launch_gmr` | `gmr` 的兼容别名，不能与 `gmr` 填写矛盾的值 |
| `gmr_device` 等 `gmr_` 前缀参数 | 原参数名的兼容别名；新命令直接写 `device` 等原名即可 |

独立 `retarget.launch.py` 始终启动 GMR，不使用上表的联合启动开关。
GMR 发布到 `/noetix/gmt`；启动 GMR 本身不使能电机，也不自动激活策略。
同一话题只运行一个 GMR 发布实例。

联合启动按 `robot_name` 自动选择重定向预设：

| 部署机型 `robot_name` | GMR 预设 `robot` |
| --- | --- |
| `n3` | `n3` |
| `bumi2`、`bumi3`、`bumi4340` | `bumi3` |

联合启动一般省略 `robot`；若填写，必须与自动选择的预设一致。
独立启动则用 `robot` 选择 `n3`、`bumi3`、`g1`、`e1`。
N3 内置 IK 配置目前支持 Pico，其他设备需要提供匹配的 `ik_config`。

### 设备和 IK 参数

以下 GMR 参数既可用于独立启动，也可直接追加到 `real.launch.py` / `sim.launch.py`。
联合启动未填写时沿用 GMR 的默认值。

| 参数 | GMR 默认值 | 含义和作用 |
| --- | --- | --- |
| `device` | `pico` | 动捕设备：`pico`、`xsens`、`optitrack`、`fzmotion` |
| `robot` | 独立启动为 `n3`；联合启动自动选择 | 重定向目标机器人预设 |
| `server_ip` | 空 | OptiTrack/Motive 或 FZMotion 服务器 IP |
| `client_ip` | 空 | OptiTrack 本机接收网卡 IP；OptiTrack 必须同时填写两个 IP |
| `xsens_port` | `9763` | Xsens MVN UDP 接收端口 |
| `subject_filter` | 空 | FZMotion 人物名称过滤；空表示接受任意跟踪人物 |
| `position_scale` | `0.001` | FZMotion 位置单位转为米的比例；毫米数据使用 0.001 |
| `human_height` | Pico 为 `1.6`，其他设备为 `1.8` | 操作者身高，单位米，用于人体到机器人比例缩放 |
| `model_path` | 预设中的 MJCF | 自定义机器人 MJCF 的绝对路径；联合打开的查看器也使用该路径 |
| `ik_config` | 设备与机型对应的 JSON | 自定义 IK 目标、权重和人体比例配置的绝对路径 |
| `ik_damping` | Xsens/FZMotion 为 `1.0`，其他为 `0.5` | IK 求解正则化阻尼，增大可减小不稳定解，但会影响跟踪 |
| `warmup_iterations` | `1000` | 首次有效动捕帧的 IK 预热迭代次数；0 表示不预热 |
| `offset_to_ground` | `true` | 启用动捕贴地与已配置的机器人脚底高度对齐 |
| `pico_startup_timeout` | `30.0` | 等待首次 Pico 身体数据的超时，单位秒 |
| `normalize_initial_yaw` | `false` | 是否将 Pico 启动时的初始航向归零 |
| `spine_joint` | 按设备/机型选择 | 施加额外旋转的源脊柱骨骼名；G1 非 Pico 默认 `Spine1`，其余默认空 |
| `spine_deg` | 通常 `0`；选中脊柱时默认 `6` | 对所选源骨骼施加额外旋转，单位度；0 关闭 |

### 输出、滤波和显示参数

| 参数 | 默认值 | 含义和作用 |
| --- | --- | --- |
| `publish_hz` | `50.0` | GMR 话题最大发布频率，单位 Hz；仅发布新采样，与策略推理频率独立 |
| `frame_timeout` | `0.5` | 连续无新 IK 帧达到该秒数后告警并暂停发布；恢复后速度差分重新初始化 |
| `interpolation_delay_ms` | `0.0` | 输出插值等待延迟，单位毫秒；需小于 `frame_timeout` 对应时长 |
| `pelvis_z_offset` | 通常 `0`；OptiTrack 按机型 | 发布时给骨盆高度增加的偏移，单位米 |
| `root_z_offset` | `0.0` | 计算输出速度前，给根节点高度增加的偏移，单位米 |
| `root_xy_scale` | `1.0` | 相对首次位置缩放根节点水平位移 |
| `motion_scale` | `1.0` | 关节角幅度倍率；小于 1 缩小动作，大于 1 放大动作 |
| `max_joint_speed` | `0.0` | 输出关节速度上限，单位 rad/s；0 关闭限速 |
| `lock_root_tilt` | `false` | 保留根节点 yaw，去掉 roll/pitch 倾斜 |
| `lin_vel_alpha`、`ang_vel_alpha` | `1.0` | 线/角速度 EMA 新样本权重，范围 0～1；越小越平滑，1 不平滑 |
| `lin_vel_max`、`ang_vel_max` | `0.0` | 线/角速度尖峰阈值，单位 m/s、rad/s；超限时保留此前速度，0 关闭 |
| `visualize` | `false` | 额外打开 MuJoCo 查看窗口 |
| `viewer_width`、`viewer_height` | `960`、`720` | 随 GMR 启动的查看窗口尺寸，单位像素 |

只打开查看器，订阅已有动捕数据，不连接动捕设备：

```bash
ros2 launch noetix_gmr viewer.launch.py robot:=n3
```

独立查看器参数为 `robot:=n3`、`model_path:=绝对路径`、`topic:=/noetix/gmt`、
`width:=960`、`height:=720`。此处窗口尺寸参数是 `width` / `height`。
鼠标旋转、平移和缩放，R 重置视角，F 切换跟随，Esc 关闭。

## 6. 参数查询和运行状态

查看各入口实际支持的参数：

```bash
ros2 launch noetix_bringup real.launch.py --show-args
ros2 launch noetix_gazebo sim.launch.py --show-args
ros2 launch noetix_mujoco sim.launch.py --show-args
ros2 launch noetix_policy policy.launch.py --show-args
ros2 launch noetix_gmr retarget.launch.py --show-args
ros2 launch noetix_gmr viewer.launch.py --show-args
```

查看运行状态、目标发布频率和动捕发布频率：

```bash
ros2 topic echo /noetix/control_state
ros2 topic echo /noetix/policy_state
ros2 topic hz /noetix/low_cmd
ros2 topic hz /noetix/gmt
ros2 run plotjuggler plotjuggler
```

`/noetix/low_cmd` 由当前姿态或策略控制节点发布，测到的频率需要结合当前模式判断。
策略节点在加载、切换、参考状态变化、周期性能统计或错误时打印信息。

GMR仅输出启动、连接状态变化和必要异常提示，不再提供逐帧追踪、周期耗时统计参数。

## 7. 根目录 SH 脚本使用方法

两个脚本与本 README 位于同一目录。首次使用先进入工作区：

```bash
cd /home/byf/scy/noetix_ws_c++
chmod +x build.sh start.sh
```

### 第一步：编译部署工程

```bash
./build.sh
```

脚本自动加载 ROS 2 Humble，以 Release 模式编译真机底层、C++策略、GMR 和 MuJoCo。
**不会扫描或编译 Gazebo，板子不需要安装 Gazebo。**
结果写入当前项目的 `build/`、`install/`、`log/`；这里的“打包”指编译安装，
不生成压缩包或跨架构安装包。系统依赖和 ONNX Runtime SDK 需按第1节预先安装。

板子内存有限时，可以减少同时构建的包数；已有完整安装后，也可以只更新策略：

```bash
./build.sh --parallel-workers 2
./build.sh --packages-select noetix_policy
```

### 第二步：选择真机或 MuJoCo 启动

真机启动底层、姿态节点、手柄和策略切换器：

```bash
./start.sh real robot_name:=n3 use_rviz:=false
```

真机同时连接 Pico，启动 GMR 和 MuJoCo 动捕参考查看器：

```bash
./start.sh real robot_name:=n3 use_rviz:=false \
  device:=pico human_height:=1.78 publish_hz:=50.0 visualize:=true
```

这里的 `visualize:=true` 仅显示动捕参考，实际控制对象仍是真机。
需要 MuJoCo 物理仿真时，使用 `sim` 模式：

```bash
# MuJoCo物理仿真和策略切换器。
./start.sh sim robot_name:=n3
# 不打开MuJoCo窗口。
./start.sh sim robot_name:=n3 gui:=false
# MuJoCo物理仿真，同时连接Pico动捕。
./start.sh sim robot_name:=n3 device:=pico human_height:=1.78 publish_hz:=50.0
```

| 写法或参数 | 含义 |
| --- | --- |
| `real` | 真机底层和策略切换器；省略模式时默认使用它 |
| `sim` | MuJoCo物理仿真和策略切换器，不启动真机硬件或Gazebo |
| `robot_name:=n3` | 选择底层机器人资源；不会自动更换策略模型 |
| `use_rviz:=false` | 真机模式关闭RViz，减少板子显示开销 |
| `gui:=false` | MuJoCo模式关闭物理仿真窗口 |
| `device:=pico` | 联合启动Pico动捕重定向 |
| `human_height:=1.78` | 操作者身高，单位米 |
| `publish_hz:=50.0` | GMR参考发布上限，不改变策略推理频率 |
| `visualize:=true` | 额外打开GMR的MuJoCo参考查看器 |
| `joystick_config:=/绝对路径/joystick.yaml` | 指定策略切换器的手柄配置 |

`start.sh` 自动加载 ROS 和本工作区的 `install/setup.bash`，不需要另开终端启动策略。
真机模式的 `joystick_config` 传给策略，其他参数传给 `real.launch.py`；
仿真模式的参数传给 `noetix_mujoco/sim.launch.py`，默认 `launch_policy:=true`。
其他可用参数见前文相应launch参数表。

### 第三步：手柄操作与退出

脚本只启动进程，不自动使能机器人或激活策略。默认键位：
**7使能并进入默认姿态、6阻尼3秒后失能、6+7立即失能、3电机清错、2站立、0行走、4进入GMT、4+5进入motion**。
模型对应机型见第4节。

在脚本终端按 **Ctrl+C** 退出。真机模式会同时停止本脚本启动的底层和策略；
其中一个launch提前退出时，也会清理另一个。
真机与MuJoCo仿真每次选择一种模式运行。

查看脚本帮助：

```bash
./build.sh --help
./start.sh --help
```
