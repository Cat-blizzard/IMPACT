# GPU P4 诊断与单次开发验证（2026-09-17）

## 结论

当前证据将 GPU 问题拆成了两个独立部分。

1. 已确认并修复的首因是 Gazebo GPU 渲染进程在退出时段错误，不是 ExternalNav、MAVROS 或任务节点先崩溃。该问题可在不包含 IMPACT、ROS、MAVROS 和 ArduPilot 的最小 Gazebo 相机世界中复现；GPU D3D12 路径退出为 139，空世界和 CPU llvmpipe 相机路径退出为 0。
2. 历史 `VisOdom: not healthy`、解锁拒绝和 `Gyros inconsistent` 不能据此归为同一原因。冻结提交 `b3ada48` 的最终 GPU 不解锁冒烟和一次 P4 开发飞行均未复现这些飞控健康故障，但一个成功飞行样本不能证明历史问题已经永久消失。

因此，之前的“GPU P4 不行”至少包含一个确定的退出基础设施故障。该故障现已被修复，当前版本能够完成严格 GPU 冒烟和一次完整 P4；尚未建立三次独立冷启动 GPU P4 验证组，不得宣称 GPU 稳定性验收完成。

## 冻结对象

| 项目 | 值 |
|---|---|
| 验证源码提交 | `b3ada485aa1b23092ddcc372e05891ce45c5a294` |
| 源码哈希 | `cba2bc5857068e3fe8b3a7a0fc015f36a6bda78f3438ce91ffc853159ad1d849` |
| 安装目录 | `/home/ld666/impact-build/install` |
| 构建清单 SHA-256 | `8c689b1f40791e483baee74cbdfecc6239f76f333cf1f9c14273cf0ffabf1391` |
| 渲染器 | `D3D12 (NVIDIA RTX PRO 6000 Blackwell Workstation Edition)`，硬件加速 |
| WSL 内核 | `6.6.114.1-microsoft-standard-WSL2` |
| Gazebo | `gz-sim 8.15.0` |
| EGL 退出保护库 SHA-256 | `56fbf9ea817b45ed44b17b85f5bc2eb41c8cbfdd3815ebc5c6ec46dbd008cf10` |

构建清单绑定实际安装树、Git 提交和 `src/scripts/config` 源码哈希。运行入口在安装树或清单不匹配时拒绝启动。

## 首因证据

### 1. 全栈 GPU 冒烟对照

在退出保护前，`p4_gpu_37ffc80_no_arm_diag02` 的运行期数据链路已经满足严格条件：ExternalNav 连续健康、飞控已使用 ExternalNav、DataFlash VISP 连续且没有关键 FCU 故障。唯一阻塞项是 Gazebo 在 `/server_control` 返回成功后退出 139。

同一代码路径的 CPU 控制样本 `p4_cpu_37ffc80_no_arm_diag_cleanup01` 正常退出。说明当时的确定差异位于 GPU 渲染销毁路径，而不是任务、ExternalNav 或统一清理的通用逻辑。

### 2. 最小复现

最小复现只运行 Gazebo 自带相机传感器世界：

| 条件 | 结果 |
|---|---|
| GPU D3D12 + 相机传感器 | 退出 139 |
| GPU D3D12 + 空世界 | 退出 0 |
| CPU llvmpipe + 相机传感器 | 退出 0 |
| GPU D3D12 + Ogre1 相机 | 相机有数据，退出仍为 139 |
| GPU D3D12 + 跳过 `eglTerminate` | 相机有数据，退出 0 |

GDB 原件保留在 `/var/tmp/impact-gz-gdb-FobinM/gdb.log`。停止线程位于 `gz::sim::v8::systems::SensorsPrivate::Stop()` 并等待渲染线程；渲染线程在 `__nptl_deallocate_tsd` 期间跳转到无效地址，进程中仍有 `libnvwgf2umx.so` 的 WSL NVIDIA 驱动线程。

该栈与 Microsoft WSLg 已公开的 EGL/D3D12 TLS 析构问题一致：`eglTerminate` 卸载 D3D12 模块后，线程退出仍调用已卸载模块中的 TLS 析构函数。Gazebo Sensors 系统的实现也显示停止路径会等待渲染线程，并在渲染线程返回前销毁渲染资源。参考：

- <https://github.com/microsoft/wslg/issues/1497>
- <https://github.com/gazebosim/gz-rendering/issues/662>
- <https://github.com/gazebosim/gz-sim/blob/main/src/systems/sensors/Sensors.cc>

### 3. 修复边界

新增 `libimpact_egl_terminate_guard.so`，仅在同时满足以下条件时注入 Gazebo 进程：

- 配置为 `server_gpu`；
- `glxinfo` 确认为 NVIDIA 硬件渲染且不是 llvmpipe/softpipe；
- 渲染器为 D3D12；
- 内核标识为 Microsoft/WSL。

保护库只在进程退出时拦截 `eglTerminate`，将 D3D12 模块保留到进程退出，由操作系统回收资源。它不进入 ROS、MAVROS、ExternalNav、SITL 或任务进程，也不改变定位、飞控、传感器和任务参数。运行记录保存保护库路径及哈希；预期启用时若日志没有保护标记，严格冒烟失败。

清理流程先请求 Gazebo `/server_control stop:true`，有界等待后才向本轮 Gazebo 进程组发送 TERM。退出码、强制终止、残留进程和崩溃标记仍是硬检查，没有把 139 加入允许列表。

## 最终 GPU 不解锁冒烟

运行目录：`experiments/results/external_nav/p4_gpu_b3ada48_no_arm_smoke_final`

结果为 `PASS`：

- 45.005 秒窗口内没有任务节点、ARM 或 TAKEOFF 证据，FCU 始终未解锁；
- `/localization/odom` 和 ExternalNav 输出各 458 条，约 10.18 Hz，时间戳无回跳；
- ExternalNav 状态 230 条全部健康；
- 状态与输出各只有当前适配器一个发布端，MAVROS 订阅端身份和 GID 可解析；
- FCU DataFlash VISP 为 664 条，ROS 输出为 676 条，接收比例 98.22%，没有大于等于 300 ms 的间隔；
- rosbag 可读，90,814 条消息，关键话题有数据；
- Gazebo 保护标记存在，全部进程退出码为 0，无强制 KILL、无残留和段错误。

## 单次 GPU P4 开发飞行

运行目录：`experiments/results/external_nav/p4_gpu_b3ada48_dev_flight01`

结果为 `PASS`：

- GPS 禁用且 EKF 源绑定 ExternalNav；
- 健康门禁通过后只发送一次 ARM 请求；
- 起飞、悬停、矩形四点、返回、降落完成；
- 任务结果成功，终止单独确认：`LAND`、已解除武装、状态年龄 0.078 秒；
- 890 个匹配样本的 ATE RMS 为 0.005599 m，最大位置误差 0.015715 m，最终误差 0.006728 m；
- rosbag 可读，119,420 条消息；DataFlash 可读，VISP 885/ROS 输出 890；
- 运行审计确认唯一任务节点、唯一 ExternalNav 发布者、正确 MAVROS 订阅和真值隔离；
- 所有进程退出码为 0，无残留、无段错误。

该样本保留一个解锁前的 VISP 接收间隔 0.323 秒。任务期间没有 `VisOdom`、`Gyros inconsistent` 或 `Potential Thrust Loss` 告警，诊断结果也没有首个 FCU/DataFlash 故障。该间隔不影响本次任务判定，但在重复验证中需要继续监测。

MAVROS 在 GPU 冒烟和飞行中仍记录了 `Wrong FCU time` / `Time jump detected`。既有 CPU 成功样本也有相同类型记录，因此它不是 GPU 独有证据；本次未观察到它引发定位时间戳回跳、ExternalNav 不健康或任务失败。当前只能将其列为独立的时间同步观察项，不能据此解释历史 `VisOdom` 故障。

## 尝试清单

| 尝试 | 结果 | 主要证据 |
|---|---|---|
| `p4_gpu_37ffc80_no_arm_diag01` | FAIL | 运行健康；Gazebo TERM 后退出 139；另有一个 VISP 间隔超限 |
| `p4_gpu_37ffc80_no_arm_diag02` | FAIL | 运行数据严格通过；`server_control` 成功后 Gazebo 仍退出 139 |
| `p4_cpu_37ffc80_no_arm_diag_cleanup01` | PASS | CPU 控制路径清理正常 |
| 最小 GPU 相机世界 | FAIL | 无 IMPACT/ROS/飞控仍退出 139 |
| 最小 CPU 相机/最小 GPU 空世界 | PASS | 将故障限制到 GPU 渲染传感器销毁 |
| 最小 GPU 相机 + EGL 保护 | PASS | 相机有数据，保护标记存在，退出 0 |
| `p4_gpu_37ffc80_no_arm_eglguard01` | PASS | 开发全栈冒烟验证保护有效 |
| `p4_gpu_b3ada48_no_arm_smoke_final` | PASS | 冻结提交严格不解锁冒烟 |
| `p4_gpu_b3ada48_dev_flight01` | PASS | 冻结提交一次完整 P4 开发飞行 |

## 复现命令

以下命令在 `zyc111`、用户 `ld666`、仓库 `/home/ld666/projects/IMPACT` 中执行：

```bash
export IMPACT_BUILD_ROOT=/home/ld666/impact-build
export IMPACT_INSTALL=/home/ld666/impact-build/install
export IMPACT_BUILD_MANIFEST=/home/ld666/impact-build/build-manifest.json
export ARDUPILOT_ROOT=/home/ld666/impact-deps/ardupilot
export ARDUPILOT_GAZEBO_ROOT=/home/ld666/impact-deps/ardupilot_gazebo

bash scripts/impact.sh build
bash scripts/impact_test.sh /home/ld666/projects/IMPACT /home/ld666/impact-build

bash scripts/run_p4_external_nav.sh \
  --profile server_gpu \
  --smoke-only \
  --observation-seconds 45 \
  --run-dir experiments/results/external_nav/<new-gpu-smoke-id>

bash scripts/run_p4_external_nav.sh \
  --profile server_gpu \
  --minimum-eval-duration 70 \
  --run-dir experiments/results/external_nav/<new-gpu-p4-id>
```

每次必须使用新目录，并从本任务全部进程已经退出的状态开始。不得用后一次覆盖前一次，也不得自动重试 ARM 或挑选成功样本。

## 限制与下一步

- 当前只有一次冻结版本 GPU P4 开发飞行通过，不是三次独立冷启动验收，也不是长期稳定性统计。
- EGL 保护是针对 WSL D3D12 已知销毁缺陷的进程退出规避，不是上游 Mesa/WSLg/Gazebo 修复；系统组件升级后应重新做最小复现，评估是否可以移除。
- 历史 `VisOdom: not healthy` 根因没有被本次退出修复证明消除；若再次出现，仍需按测量时间、FCU 接收时间、坐标变换和 DataFlash 对齐分析。
- CPU 主验证结论、旧 P5 FAIL、1.2 m `UNVERIFIED_FLIGHT`、阶段 A/B 和正式矩阵状态均未被本轮改变。
- 若恢复 GPU 验收，应先冻结同一提交、依赖和参数，预先登记三次冷启动安排；任一失败即停止并分析，不能重跑凑数。

轻量索引见 `evidence/P4_GPU_20260917/validation-index.json`，大型产物哈希见 `evidence/P4_GPU_20260917/LARGE_ARTIFACTS.sha256`。
