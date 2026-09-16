# CPU 软件渲染 P4 最终开发验收（2026-09-17）

## 结论

冻结代码提交 `b88288f912797538dc0688b50a56fbc3c3fc668f` 在同一安装树、依赖清单、场景、参数和 CPU 软件渲染配置下完成：

- 一次严格不解锁冒烟：PASS；
- 三次独立冷启动 P4 正常任务：全部 PASS；
- 每次 rosbag 与 DataFlash 均可读取，任务与终止结果分别确认；
- 每次清理均无强制杀进程、无残留、无本轮 Gazebo 段错误或 core dump；
- 完整 Python 回归 `196 passed`，ROS 合约检查 PASS。

这三次属于开发验收，不是长期可靠性或算法收益的统计证明。GPU P4 仍是独立未解决问题；本报告不包含阶段 A、P5、自主恢复、正式矩阵或实机验收。

## 冻结对象

| 项目 | 冻结值 |
|---|---|
| 验证代码提交 | `b88288f912797538dc0688b50a56fbc3c3fc668f` |
| 安装目录 | `/home/ld666/impact-build/install` |
| 构建清单 | `/home/ld666/impact-build/build-manifest.json` |
| 源码哈希 | `934c98f511d1b5809c6dd4071f38a858238ba88ace6f3eabbf8218927f3f7d25` |
| 构建清单哈希 | `eba577bce534c3dfd081c2fdf9fca64d126b0be7797f6683a39b867a6dbc34ed` |
| 运行依赖清单哈希 | `4d1a13002201e1ec3500a24d2c4fc90a9059bd67cfee4dac0fa33c2a0a860388`，四组一致 |
| P4 世界哈希 | `758e70987bbd60ac60198d2654b892b476b1bf7f534935b0eeae2f7fe6dcac59` |
| 飞控参数哈希 | `2d17c98d4e1df36c92aabf0e5efcfac72dfcc72c6027f3a2ae1a24e6e5ac2ece` |
| 传感器桥配置哈希 | `b235ba27a8f5df24d2a83827de823d062ea5ca892fd6c89ed2f7604e331a8f3f` |
| 传感器桥随机种子 | `20260822` |
| 渲染配置 | `local_cpu`；Mesa llvmpipe 23.2.1；`Accelerated: no` |
| GPS | `GPS_TYPE=0`、`SIM_GPS_DISABLE=1`，并核对第二 GPS 关闭 |

每次运行都先由 `verify_runtime_build.py` 核对 Git、源码哈希、安装树文件和实际 console entrypoint/module 来源；不匹配即在启动 SITL 前拒绝。四次运行的 `runtime-build-verification.json` 哈希均为 `cc174b4d00d32b1c0e134ace4c9fbc3b00f65c4f13060ce4cd2e39e0e916acd9`。

Gazebo/ArduPilot 自身没有在本入口暴露额外统一随机种子；可控的传感器噪声桥种子已冻结。该边界保留为复现限制，不虚构不存在的种子字段。

## 验收结果

| 运行 ID | 模式 | 结果 | 关键证据 |
|---|---|---|---|
| `p4_cpu_b88288f_no_arm_smoke_final` | 45 s 不解锁冒烟 | PASS | mission 节点不存在；ARM/TAKEOFF 证据为零；FCU 全程未解锁；ExternalNav 健康 228/228；DataFlash VISP 486；清理 PASS |
| `p4_cpu_b88288f_group01_run01` | P4 冷启动 1 | PASS | 67.501 s；一次 ARM；矩形与返回完成；落地并解除武装；ATE RMS 0.005263 m；VISP 667 |
| `p4_cpu_b88288f_group01_run02` | P4 冷启动 2 | PASS | 68.501 s；一次 ARM；矩形与返回完成；落地并解除武装；ATE RMS 0.005107 m；VISP 660 |
| `p4_cpu_b88288f_group01_run03` | P4 冷启动 3 | PASS | 68.101 s；一次 ARM；矩形与返回完成；落地并解除武装；ATE RMS 0.005268 m；VISP 665 |

三次任务的终止证据均为新鲜 FCU 状态、`mode=LAND`、`armed=false`，不是仅凭 LAND 请求判定。三次定位最大误差分别为 0.014582 m、0.011374 m、0.011609 m；独立定位评估均为 PASS。仿真真值只进入独立评估和 bag 记录，没有进入 ExternalNav、任务或飞控导航输入；运行图审计确认真值订阅隔离。

冒烟窗口内实测 `/localization/odom` 325 条、ExternalNav 输出 325 条、FCU state 32 条；ExternalNav 状态无不健康样本，FCU 无解锁样本。完整运行记录中 ROS ExternalNav 输出 487 条、DataFlash VISP 486 条，飞控实际接收比例 99.79%，无 300 ms 以上 VISP 接收间隔。

三次 P4 的 bag 关键话题计数分别为：

| 运行 | odom | ExternalNav out | FCU state | ExternalNav status | truth |
|---|---:|---:|---:|---:|---:|
| run01 | 668 | 668 | 74 | 476 | 4346 |
| run02 | 667 | 665 | 74 | 465 | 4341 |
| run03 | 671 | 671 | 74 | 474 | 4340 |

run01/run02 的 DataFlash VISP 最大接收间隔为 0.145 s/0.148 s，均无 300 ms 以上间隔。run03 在任务节点启动约 20 s 前、ARM 约 26 s 前出现一次 0.31988 s 的启动间隔；之后 EKF 初始化、健康门禁重新建立、解锁前检查和完整任务均通过，且没有 FCU fault。该瞬态被保留，不表述为“三次全程零间隔”。

## 修复范围

从远端基线 `865990a` 到验证提交 `b88288f` 的修改集中于 P4 启动、运行隔离、证据和安全终止：

- 启动前绑定 Git、源码哈希、安装树及实际入口，拒绝陈旧或错误安装；
- 不解锁冒烟使用独立入口，不启动 mission，不发送 ARM/TAKEOFF；
- ROS 图查询设置硬超时并禁用 daemon 缓存，保留端点 GID、participant GID、节点和 namespace；
- 分别验证 ExternalNav 唯一发布端和 MAVROS 实际订阅端，未知身份不能直接放行；
- ExternalNav 输出队列深度调整为 1，避免陈旧测量积压送入 MAVROS；
- 用 DataFlash VISP 验证飞控真实接收，而非只依据 ROS 发布计数；
- P4 解锁前要求连续 ExternalNav 与 FCU prearm/vision 健康，只发一次 ARM 请求；
- 故障后按新鲜飞行状态进入适当终止流程，任务失败与终止确认分开；
- 首次失败、阶段、命令、行号和退出码独立记录，清理错误不会覆盖首因；
- 清理等待每个进程组退出，记录退出码、强制信号、残留与崩溃标记；
- 只忽略 ROS Humble 在已关闭 context 上的已知 shutdown 竞态，正常运行异常仍会抛出。

## 全部尝试

截至本报告，`experiments/results/external_nav/` 下的历史和本轮运行均已盘点。早期样本如下；它们没有运行时安装树绑定证据，或不属于最终提交，因此只保留作历史诊断，不计入最终组：

| 运行 ID | 原始结果 |
|---|---|
| `frozen_09f8dcf/p4_cold_start_01`—`03` | 历史冻结组三次 PASS |
| `p4_health_gate_dev_01`、`02` | 启动/健康门禁开发样本，未形成任务结果 |
| `p4_health_gate_dev_03`—`05` | 三次 PASS |
| `p4_health_gate_dev_06` | FAIL：未达到起飞高度 |
| `p4_health_gate_dev_07` | FAIL：飞行中 VisOdom 不健康，终止已确认 |
| `p4_frozen_dev_01` | FAIL：飞行中 VisOdom 不健康，终止已确认 |
| `p4_20260915T233208Z_136095` | FAIL：Potential Thrust Loss，终止已确认 |
| `p4_20260915T233437Z_137545` | PASS |
| `p4_20260915T233658Z_139221` | PASS |
| `p4_20260915T233919Z_140876` | FAIL：Potential Thrust Loss，终止已确认 |
| `p4_20260915T234156Z_142309` | FAIL：Potential Thrust Loss，终止已确认 |
| `p4_20260916T052630Z_147532` | FAIL：VisOdom 不健康，终止已确认 |
| `p4_20260916T054248Z_157661` | PASS |
| `p4_20260916T054523Z_159319` | FAIL：VisOdom 不健康，终止已确认 |

本轮安装树绑定、严格冒烟及退出验收的收敛尝试如下。失败和中间版本均保留，没有删除，也没有与最终组混算：

| 运行 ID | 结论 | 后续单一处理 |
|---|---|---|
| `p4_cpu_7665bcb_group01_run01` | FAIL：ARM 被拒，VisOdom 不健康 | 增加飞控实际接收诊断，收紧 ExternalNav 输出队列 |
| `p4_cpu_81fd1f8_no_arm_smoke_01` | ERROR：未知 MAVROS 端点导致图审计拒绝 | 依据 DDS participant GID 解析端点身份 |
| `p4_cpu_ae8dbab_no_arm_smoke_01` | ERROR：探针依赖瞬态 odometry/in；清理日志含 static transform `-11` | 改用 MAVROS state 身份交叉检查，继续保留异常样本 |
| `p4_cpu_dfab15c_no_arm_smoke_01` | PASS，但不是最终提交 | 未计入最终验收组 |
| `p4_cpu_35763a3_no_arm_smoke_final` | ERROR：图发现快照漏掉 MAVROS 订阅/state | 为图发现设置有界 spin 时间 |
| `p4_cpu_d921072_no_arm_smoke_final` | PASS，但不是最终提交 | 未计入最终验收组 |
| `p4_cpu_d921072_group01_run01` | 任务、定位、落地 PASS；清理 FAIL | 修复 ROS context 关闭竞态后新建验证组，旧结果不计 PASS |
| `p4_cpu_b88288f_no_arm_smoke_final` | 最终冒烟 PASS | 计入最终验收 |
| `p4_cpu_b88288f_group01_run01` | 最终 P4 PASS | 计入最终验收 |
| `p4_cpu_b88288f_group01_run02` | 最终 P4 PASS | 计入最终验收 |
| `p4_cpu_b88288f_group01_run03` | 最终 P4 PASS，保留启动期 VISP 瞬态 | 计入最终验收并列为限制 |

每个失败后均先形成新证据或最小修复，再建立新提交/验证组；没有通过自动反复 ARM 或挑选成功样本凑三次 PASS。

## 复现命令

以下命令必须在 `zyc111`、用户 `ld666`、仓库 `/home/ld666/projects/IMPACT` 中运行。先重建并生成与当前代码一致的 `/home/ld666/impact-build/build-manifest.json`，再设置：

```bash
export IMPACT_INSTALL=/home/ld666/impact-build/install
export IMPACT_BUILD_MANIFEST=/home/ld666/impact-build/build-manifest.json
export ARDUPILOT_ROOT=/home/ld666/impact-deps/ardupilot
export ARDUPILOT_GAZEBO_ROOT=/home/ld666/impact-deps/ardupilot_gazebo
```

严格不解锁冒烟：

```bash
bash scripts/run_p4_external_nav.sh \
  --profile local_cpu \
  --smoke-only \
  --observation-seconds 45 \
  --run-dir experiments/results/external_nav/<new-smoke-run-id>
```

每次 P4 必须使用新的运行目录，并从全部进程已退出的冷状态单独启动：

```bash
bash scripts/run_p4_external_nav.sh \
  --profile local_cpu \
  --minimum-eval-duration 70 \
  --run-dir experiments/results/external_nav/<new-p4-run-id>
```

回归与 ROS 合约：

```bash
bash scripts/impact_test.sh \
  /home/ld666/projects/IMPACT \
  /home/ld666/impact-build
```

完整大型数据保留在服务器。轻量机器索引见 `evidence/P4_CPU_20260917/validation-index.json`，内容哈希见 `evidence/P4_CPU_20260917/LARGE_ARTIFACTS.sha256`。

## 已知限制与阶段 A 条件

- GPU P4 未通过状态和既有故障证据继续保留；CPU 结果不能替代 GPU 验收。
- 旧 P5 FAIL 与 1.2 m 观察点 `UNVERIFIED_FLIGHT` 保持不变。
- run03 的启动期 VISP 间隔需继续观察；三次成功不证明 ExternalNav/EKF 根因在所有环境已消失。
- 本轮未验证共享地图内容、EGO 碰撞地图、目标导航、完整性恢复效果或正式 120 次矩阵。
- 本轮为 SITL，不能外推为实机验收。

进入下一目标的条件是：以本冻结 CPU P4 基线为起点，单独定义阶段 A 验收组，先运行 `normal / baseline / seed 1000` 固定目标导航；同时解析并核查共享地图与 EGO 碰撞地图的实际内容、坐标对齐、膨胀和可达性，分别验证正常到达与失败终止。阶段 A 不能改名替代旧 P5，也不能仅凭话题存在判地图正确。

本报告之后的文档/证据索引提交不改变已验证源码或安装树；实际运行提交始终是 `b88288f`。因此不为纯报告提交重跑飞行，且不得把文档提交号伪写进既有运行 manifest。
