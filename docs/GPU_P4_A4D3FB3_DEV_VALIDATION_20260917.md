# GPU P4 当前版本单次开发验证（2026-09-17）

## 结论

冻结运行提交 `a4d3fb33209b73f2a3c1514ff4709848db28d1b9` 在 WSL D3D12 NVIDIA
硬件渲染下完成：

- 45.008 秒严格不解锁冒烟：PASS；
- 一次 P4 正常飞行：PASS；
- 起飞、悬停、矩形四点、返回、LAND 和解除武装确认完整；
- rosbag、DataFlash、运行图和退出清理均通过；
- 没有 `VisOdom: not healthy`、`Gyros inconsistent`、`Potential Thrust Loss`、
  EKF variance 或 EKF failsafe；
- Gazebo 正常退出，无段错误、强制 KILL 或残留进程。

这是当前版本 GPU P4 的单次开发验证，不是三次独立冷启动验收或长期稳定性证明。

## 冻结对象

| 项目 | 值 |
|---|---|
| 运行代码提交 | `a4d3fb33209b73f2a3c1514ff4709848db28d1b9` |
| 源码哈希 | `2664b62298ef224eed706c09b26a2ca228d6111c13c3e667f7668fdcc6521862` |
| 安装目录 | `/var/tmp/impact-1000-846fb8ab40/install` |
| 构建清单 SHA-256 | `b1b4d492fb8b4d020c69c01f2b4218daaa7d332d5f1f44621c19676fd1d54615` |
| 安装文件核验 | 1667 个文件；实际 ROS/Python 入口核对通过 |
| ArduPilot | `2a3dc4b7bf2507120f7378a7b2fde73185e0c325`；二进制 `754e5eaf...` |
| ArduPilot Gazebo 插件 | `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`；二进制 `e5d11ebd...` |
| GPU 运行依赖清单 SHA-256 | `17a675003b21230c2c194df158b0a73b613c0b3ec2c43814a3e10fb325e0590c`，两轮一致 |
| 渲染器 | `D3D12 (NVIDIA RTX PRO 6000 Blackwell Workstation Edition)`；硬件加速 |
| 驱动 | NVIDIA `596.49` |
| Gazebo | `gz-sim 8.15.0` |
| EGL 退出保护库 SHA-256 | `1a972bda118a909950ade41e51f1c6216b500c39cf21584af5a888b9e64010a6` |

世界、飞控参数、模型、FAST-LIO 和传感器桥分别为
`758e709...`、`2d17c98...`、`df69169...`、`a9c1ed7...` 和 `b235ba2...`，与同提交
CPU 验证使用的输入一致；GPU 清单只额外包含本次安装树中的 EGL 退出保护库。

## 不解锁冒烟

运行目录：
`experiments/results/external_nav/p4_gpu_a4d3fb3_group01_smoke45`

- 实际观察窗口 45.007674 秒；
- `/localization/odom` 432 条，ExternalNav 状态 229 条；
- ExternalNav 229/229 健康，0 条不健康；
- FCU 全程未解锁，没有 ARM/TAKEOFF 或任务节点；
- ROS ExternalNav 输出 638 条，DataFlash VISP 627 条，接收比例 98.28%；
- VISP 最大接收间隔 0.167937 秒，无 300 ms 以上断档；
- bag 可读，关键话题有数据；端点唯一性、MAVROS 订阅身份和真值隔离通过；
- EGL 保护标记存在，全部受管进程正常退出。

## P4 飞行

运行目录：
`experiments/results/external_nav/p4_gpu_a4d3fb3_group01_run01`

| 指标 | 结果 |
|---|---:|
| 任务状态 | PASS |
| 任务时间 | 56.201 s |
| ARM 请求 | 1 次 |
| 终止 | LAND、fresh FCU state、disarmed |
| 终止状态年龄 | 0.084934 s |
| 匹配定位样本 | 770 |
| ATE RMS | 0.004411 m |
| 最大位置误差 | 0.011596 m |
| 最终位置误差 | 0.004614 m |
| DataFlash VISP | 773 |
| VISP 最大接收间隔 | 0.164943 s |
| 300 ms 以上 VISP 间隔 | 0 |

bag 中 `/localization/odom` 与 `/uav1/mavros/odometry/out` 各 778 条、FCU state 86 条、
ExternalNav 状态 470 条、评估真值 4843 条。任务只发送一次 ARM 请求；GPS 与第二 GPS
关闭，EKF 源为 ExternalNav。真值只供独立评估，未进入定位、ExternalNav 或飞控输入。

## 保留观察项

冒烟出现一次 MAVROS `Wrong FCU time`；飞行出现一次 `Wrong FCU time` 和一次
`Time jump detected`。定位及 ExternalNav 时间戳没有回跳，任务和飞控接收连续性均通过；
同类信息也存在于 CPU 成功样本，因此本轮不将其归为 GPU P4 失败，也不声称其根因已解决。

本次没有修改代码或参数，也没有在失败后自动重试。旧 GPU 失败、旧版本单次 PASS、CPU
验证、旧 P5 FAIL、1.2 m `UNVERIFIED_FLIGHT` 和阶段 A/B 状态均保持独立。

机器索引见 `evidence/P4_GPU_A4D3FB3_20260917/validation-index.json`，大型原件哈希见
`evidence/P4_GPU_A4D3FB3_20260917/LARGE_ARTIFACTS.sha256`。后续若要宣称 GPU P4 重复
验收通过，应在该冻结配置下预先登记并完成三次独立冷启动；本次 run01 可以作为该组第一轮，
但任何后续失败必须保留并停止该组分析，不能选择性补跑。
