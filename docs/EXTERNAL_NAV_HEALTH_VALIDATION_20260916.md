# ExternalNav/EKF 本轮验证摘要

更新：2026-09-16（WSL `zyc111`，本机 CPU 软件渲染）

本轮目标是连续 ExternalNav/EKF 健康检查、姿态/坐标/时间证据采集，以及固定配置下的基础飞行重复验证。健康门禁只负责在证据不健康时阻止继续推进，不能代替根因修复；本轮没有启动正式矩阵，也没有把阶段 A 或旧 P5 探索宣称为通过。

## 版本与环境

- 当前工作分支：`main`，同步远端后的合并提交由 Git 记录；本轮代码提交前的基线为 `dda6fedb092e43bda378f362c824581963a706a9`。
- 本地修复提交保留：`7c28001`、`a7d1963`；备份分支为 `backup/local-fixes-a7d1963`；原 stash 仍保留。
- ArduPilot：`/home/ld666/impact-deps/ardupilot`，版本 `2a3dc4b`。
- Gazebo 插件：`/home/ld666/impact-deps/ardupilot_gazebo`，版本 `082a0fe`。
- IMPACT 安装树：`/home/ld666/impact-build/install`；构建目录：`/home/ld666/impact-build`。
- 构建结果：14 packages finished；完整测试：`172 passed`。

## 冷启动结果

| 运行目录 | 结果 | 解释 |
|---|---|---|
| `p4_health_gate_dev_01` | 启动失败 | 旧 `ros2 topic echo --once` 探针误报 FAST-LIO 无输出；同一 rosbag 有 `/localization/odom`，保留为探针诊断证据。 |
| `p4_health_gate_dev_02` | 启动失败 | 新 odom 探针已收到数据，但 ROS CLI 诊断连接到失效 daemon；保留 traceback 和探针 JSON，未计入有效飞行。 |
| `p4_health_gate_dev_03` | P4 PASS | 任务和定位评估通过；原压缩 rosbag 无法重新打开，保留原始压缩文件和 DataFlash，未作为可回读姿态证据。 |
| `p4_health_gate_dev_04` | P4 PASS | SQLite rosbag `integrity_check=ok`，DataFlash 可读，任务成功且 FCU `LAND/disarmed`。 |
| `p4_health_gate_dev_05` | P4 PASS | SQLite rosbag `integrity_check=ok`，DataFlash 可读，任务成功且 FCU `LAND/disarmed`。 |
| `p4_health_gate_dev_06` | P4 FAIL | 首个 DataFlash/MAVROS 故障为 `Potential Thrust Loss (1)`，随后 `EKF3 lane switch 1`、`EKF variance` 和 EKF LAND；任务保留为起飞高度未达到，终态已解除解锁。 |
| `p4_health_gate_dev_07` | P4 FAIL（门禁拦截） | QoS 修复后收到 `Arm: VisOdom: not healthy`；任务在 ARM 阶段进入 `FAILSAFE_WAIT`，记录 `TASK_FAILURE` 和 `TERMINATION_CONFIRMED`，没有继续起飞。 |

因此，固定配置下有 3 次有效基础飞行重复验证（dev_03—05）；dev_01/02 是保留的启动故障样本，不混入通过率。
dev_06/07 是同一固定配置下的故障与故障保护样本，不能改写为成功飞行。
dev_07 使用最终 StatusText QoS 修复后的安装树；该修复补齐故障观测和 fail-safe 记录，不放宽任何成功条件。

## 姿态、坐标和时间证据

dev_04/dev_05 的原始定位帧是 `xq_lio_map`，ExternalNav 输出和 FCU 本地里程计帧是 `map`，子帧分别为 `livox_imu` 与 `base_link`。ExternalNav 适配器将输出时间戳改为到达时刻，原始 FAST-LIO 时间戳仍单独保存；诊断报告不把两种时钟直接当作同一时间轴。

按 rosbag 记录时间做最近邻四元数比较（仅作诊断，不证明坐标轴等价）：

- 原始定位 → ExternalNav：最大差 `3.7e-05°`（dev_04）和 `0.0044°`（dev_05）。
- 原始定位 → FCU 本地估计：最大差 `4.70°`（dev_04）和 `4.58°`（dev_05），P95 约 `3.59°` 和 `3.53°`。

这说明适配器复制姿态本身没有显示出大偏差，但 FCU 估计与原始流存在可重复的姿态差异，下一轮需用 TF 静态变换、MAVROS ODOMETRY 配置和 DataFlash `VISP/EKF` 时间顺序继续定位；不能仅凭门禁 PASS 宣布根因已修复。dev_04/dev_05 DataFlash 没有被诊断器识别的关键故障文本；历史 P5 FAIL 的 DataFlash 和 rosbag 仍是独立故障样本。

dev_06 的失败把首次异常顺序固定为“Potential Thrust Loss → EKF3 lane switch → EKF variance → EKF Failsafe LAND”，而 dev_07 的门禁样本固定为“Arm: VisOdom: not healthy → 任务失败 → 终止确认”。这两条链路分别说明飞控估计故障仍可发生，以及健康故障已能阻止任务继续；二者都不是 ExternalNav 根因已经修复的证据。

每个有效运行目录都保留：`mission-result.json`、`summary.json`、`external-nav-health-diagnostic.json`、未压缩可回读 rosbag、`sitl_runtime/logs/*.BIN`、启动探针、ROS/Gazebo 日志、依赖哈希和构建 manifest。

## 状态边界

- 旧 `P5_BASELINE_MAP_FRONTIER_EGO` 的 FAIL 记录保持不变，不能被本轮 P4 PASS 覆盖。
- `viewpoint_clearance_m=1.2` 规则仍为 `UNVERIFIED_FLIGHT`，本轮没有用 P4 方形飞行替代该验证。
- P5 运行器已保存 EGO 碰撞地图话题 `/xq/p5/ego_occupancy_inflate`，共享地图离线核对可继续；尚未据此改变旧 P5 结论。
- 阶段 A 独立目标导航协议尚未实现，本轮不进入完整性/主动恢复试跑或正式 120 次矩阵。
