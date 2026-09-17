# GPU P4 当前版本三次冷启动开发验收（2026-09-17）

## 结论

冻结运行提交 `a4d3fb33209b73f2a3c1514ff4709848db28d1b9` 在 WSL D3D12 NVIDIA
硬件渲染下完成一组固定配置验证：45 秒严格不解锁冒烟 PASS；三次独立冷启动 P4
正常任务 PASS、PASS、PASS。三次均完成起飞、悬停、矩形四点、返回、LAND 和解除武装，
定位、bag/DataFlash、端点审计及受管进程清理通过，Gazebo 无段错误、强制 KILL 或残留。

这是开发验收组，不是长期可靠性或统计稳定性证明。run02 在已经进入 LAND、接近触地时
出现一次 `EKF3 lane switch 1`，任务仍完成且终止确认完整；该事件和 VISP 间隔不会被隐去。

## 冻结对象

| 项目 | 值 |
|---|---|
| 运行代码提交 | `a4d3fb33209b73f2a3c1514ff4709848db28d1b9` |
| 源码哈希 | `2664b62298ef224eed706c09b26a2ca228d6111c13c3e667f7668fdcc6521862` |
| 安装目录 | `/var/tmp/impact-1000-846fb8ab40/install` |
| 构建清单 SHA-256 | `b1b4d492fb8b4d020c69c01f2b4218daaa7d332d5f1f44621c19676fd1d54615` |
| 安装文件核验 | 1667 个；实际 ROS/Python 入口核对通过 |
| ArduPilot | `2a3dc4b7bf2507120f7378a7b2fde73185e0c325` |
| ArduPilot Gazebo 插件 | `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` |
| GPU 运行依赖清单 SHA-256 | `17a675003b21230c2c194df158b0a73b613c0b3ec2c43814a3e10fb325e0590c` |
| 渲染器 | D3D12 NVIDIA RTX PRO 6000 Blackwell；硬件加速 |
| 驱动/Gazebo | NVIDIA 596.49 / gz-sim 8.15.0 |
| EGL 退出保护库 SHA-256 | `1a972bda118a909950ade41e51f1c6216b500c39cf21584af5a888b9e64010a6` |

世界、飞控参数、模型、FAST-LIO 和传感器桥与同提交 CPU 验证输入一致；GPU 只额外启用
安装树中的 EGL 退出保护库。

## 不解锁冒烟

运行目录：`experiments/results/external_nav/p4_gpu_a4d3fb3_group01_smoke45`

- 实际观察窗口 45.007674 秒；`/localization/odom` 432 条，ExternalNav 状态 229 条，229/229 健康；
- FCU 全程未解锁，无 ARM/TAKEOFF 或任务节点；ROS ExternalNav 输出 638 条，DataFlash VISP 627 条；
- VISP 最大接收间隔 0.167937 秒，无 300 ms 以上断档；bag 可读，关键话题、端点唯一性、MAVROS 订阅身份和真值隔离通过；
- EGL 保护标记存在，全部受管进程正常退出。

## 三次 P4 飞行

| 运行目录 | 结果 | 任务时间 | 定位/数据 | 观察项 |
|---|---|---:|---|---|
| `p4_gpu_a4d3fb3_group01_run01` | PASS | 56.201 s | ATE RMS 0.004411 m；最大 0.011596 m；VISP 773；最大间隔 0.164943 s | Wrong FCU time、Time jump；无 EKF lane switch |
| `p4_gpu_a4d3fb3_group01_run02` | PASS | 57.401 s | ATE RMS 0.006349 m；最大 0.034182 m；VISP 792；最大间隔 0.316891 s | LAND 后 55.653 s 出现 `EKF3 lane switch 1`，随后正常解除武装 |
| `p4_gpu_a4d3fb3_group01_run03` | PASS | 56.901 s | ATE RMS 0.005961 m；最大 0.015041 m；VISP 769；最大间隔 0.409859 s | VISP 间隔发生在解锁前；无 EKF lane switch |

三次均只发送一次 ARM 请求，终态均为新鲜 FCU 状态下 LAND/disarmed；GPS 与第二 GPS 关闭，
EKF 源为 ExternalNav。runtime audit 均确认唯一 ExternalNav 发布者、正确 MAVROS 订阅端、
唯一 P4 任务节点和真值隔离。真值只供独立评估，未进入定位、ExternalNav 或飞控输入。

## 观察项与解释边界

run02 的 lane switch 发生在 LAND 后低高度段。DataFlash 中两套 EKF 均 `FS=0`，ExternalNav
没有断流；切换前主核垂直速度创新短暂升高而备用核更低，解除武装后又回到默认核。这是
飞控着陆阶段 lane selection 事件，不能改写为“无飞控异常”，也没有证据把它归因于 GPU
或 VISP 间隔。run02/run03 的 0.316891/0.409859 s VISP 间隔都在解锁前，任务期没有同类断档。

MAVROS 的 `Wrong FCU time`/`Time jump detected` 也出现在 CPU 成功样本；本组没有观察到它
引起 ExternalNav 不健康、定位时间戳回跳或任务失败，根因仍单独保留。

本组没有修改代码或参数，也没有在失败后自动重试；一次缺少环境变量的入口拒绝未计入运行组。
旧 GPU 失败、CPU 验证、旧 P5 FAIL、1.2 m `UNVERIFIED_FLIGHT`、阶段 A/B 和正式矩阵状态均保持独立。

机器索引见 `evidence/P4_GPU_A4D3FB3_20260917/validation-index.json`，大型原件哈希见
`evidence/P4_GPU_A4D3FB3_20260917/LARGE_ARTIFACTS.sha256`。本报告的 GPU 结论仅限本次
开发组判据，不宣称长期稳定性、算法收益或历史 `VisOdom` 根因已解决。
