# CPU P4 当前冻结版本验收（2026-09-17）

## 结论

运行提交 `a4d3fb33209b73f2a3c1514ff4709848db28d1b9` 在固定的 CPU 软件渲染、
ArduPilot、Gazebo 插件、世界、传感器和飞控参数下完成：

- 一次严格不解锁冒烟：PASS；
- 三次独立冷启动 P4 正常任务：全部 PASS；
- 每次任务成功和落地解除武装分别确认；
- 每次 rosbag 与 DataFlash 实际可读，关键话题有数据；
- 每次退出清理无强制 KILL、无残留、无段错误或 core dump；
- 完整构建 14 个包通过，Python/ROS 回归 `336 passed`，ROS 合约检查 PASS。

这是开发验收，不是长期可靠性统计。旧 P5 FAIL、1.2 m `UNVERIFIED_FLIGHT`、GPU
遗留风险、阶段 A/B、正式矩阵和实机状态均未由本结果改变。

## 冻结对象

| 项目 | 冻结值 |
|---|---|
| 运行代码提交 | `a4d3fb33209b73f2a3c1514ff4709848db28d1b9` |
| 上游基线 | `a131a9ea19ecf7c1ebae481b6f157b97b4faa3c2` |
| 源码哈希 | `2664b62298ef224eed706c09b26a2ca228d6111c13c3e667f7668fdcc6521862` |
| 安装目录 | `/var/tmp/impact-1000-846fb8ab40/install` |
| 构建清单 | `/var/tmp/impact-1000-846fb8ab40/build-manifest.json` |
| 构建清单及安装 marker SHA-256 | `b1b4d492fb8b4d020c69c01f2b4218daaa7d332d5f1f44621c19676fd1d54615` |
| 安装文件清单 | 1667 个文件；源码、安装树和入口核对通过 |
| ArduPilot | `2a3dc4b7bf2507120f7378a7b2fde73185e0c325`；二进制 `754e5eaf...` |
| ArduPilot Gazebo 插件 | `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`；二进制 `e5d11ebd...` |
| 运行依赖清单 SHA-256 | `0b4ef308a6eea6a5ea460cefa0f537e6fe9b9c136f8d7aec0f0c328cb1c20ac1`，四组一致 |
| 渲染 | `local_cpu`；llvmpipe LLVM 15.0.7；`Accelerated: no` |
| 世界 SHA-256 | `758e70987bbd60ac60198d2654b892b476b1bf7f534935b0eeae2f7fe6dcac59` |
| 飞控参数 SHA-256 | `2d17c98d4e1df36c92aabf0e5efcfac72dfcc72c6027f3a2ae1a24e6e5ac2ece` |
| FAST-LIO 配置 SHA-256 | `a9c1ed76f6b7df45d983477f66bde06ea76234c54e15424276c1e88b4d8e99ad` |
| 传感器桥配置 SHA-256 | `b235ba27a8f5df24d2a83827de823d062ea5ca892fd6c89ed2f7604e331a8f3f` |
| 传感器桥随机种子 | `20260822` |

每次运行均先用 `verify_runtime_build.py` 核对运行 Git、源码哈希、安装 marker、安装文件
和实际 ROS/Python 入口；不匹配会在启动 SITL 前拒绝。四组最终运行的
`runtime-build-verification.json` SHA-256 均为
`85047b6a1dce35068a5ea62b29387f7d4b6bb47fd74671167d6a89c1b7d17043`。

## 修复与首因

远端 `a131a9e` 对 P4 终止期限和阶段 A 授权审计的修改方向正确，但实际运行还暴露两个
独立问题：

1. 首次严格冒烟的运行门禁、bag 和 FCU 接收均通过，但清理时 `xq_p3_evaluator` 在 ROS
   context 已关闭后抛出 Humble `RCLError`。`a4d3fb3` 只在 context 已失效时抑制该 shutdown
   race；context 仍健康时继续抛出真实异常，并补入回归测试。
2. 一次 P4 误用了 `/home/ld666/ardupilot` 的 `f9d619e` 开发版本。该版本已将
   `GPS_TYPE/GPS_TYPE2` 改名为 `GPS1_TYPE/GPS2_TYPE`，MAVROS 因此无法读取冻结门禁要求的
   `GPS_TYPE`。该轮未发送 ARM，按 FAIL 保留。最终组改用项目已有、干净且历史 PASS 对齐的
   `/home/ld666/impact-deps` 依赖，没有删除或放宽 GPS 门禁。

## 最终结果

| 运行 ID | 结果 | 任务时长 | ATE RMS | 最大位置误差 | DataFlash VISP |
|---|---|---:|---:|---:|---:|
| `p4_cpu_a4d3fb3_smoke_20260917_1` | PASS | 30.004 s 观测 | 不适用 | 不适用 | 374 |
| `p4_cpu_a4d3fb3_group01_run01` | PASS | 67.001 s | 0.005396 m | 0.013288 m | 659 |
| `p4_cpu_a4d3fb3_group01_run02` | PASS | 66.701 s | 0.005980 m | 0.015716 m | 657 |
| `p4_cpu_a4d3fb3_group01_run03` | PASS | 66.501 s | 0.005660 m | 0.014097 m | 649 |

冒烟窗口内实际订阅 `/localization/odom` 217 条、ExternalNav 状态 152 条、ExternalNav 输出
217 条和 FCU state 21 条。ExternalNav 152/152 健康，FCU 始终连接且未解锁，未发现
ARM/TAKEOFF 文本或任务节点。ROS ExternalNav 输出共 379 条，DataFlash VISP 共 374 条，
飞控接收比例 98.68%，无 300 ms 以上接收间隔。

三次 P4 都满足 GPS 及第二 GPS 禁用、EKF 源为 ExternalNav、prearm/vision 健康门禁、单次
ARM、起飞悬停、矩形四点、返回、LAND、基于新鲜 FCU 状态的 `armed=false` 确认。真值仅供
独立评估，运行图审计确认未进入定位、任务或飞控导航输入。

三次 bag 关键话题计数如下：

| 运行 | odom | ExternalNav out | FCU state | ExternalNav status | truth |
|---|---:|---:|---:|---:|---:|
| run01 | 666 | 666 | 75 | 521 | 4363 |
| run02 | 663 | 663 | 75 | 529 | 4238 |
| run03 | 655 | 655 | 74 | 514 | 4245 |

三次 DataFlash VISP 最大接收间隔分别为 0.152947 s、0.150000 s、0.150000 s，均无
300 ms 以上间隔；没有 `VisOdom: not healthy`、`Gyros inconsistent`、
`Potential Thrust Loss` 或其他关键 FCU 故障。所有受管进程均正常退出，无残留端口或本轮进程。

## 本轮全部尝试

失败、中间结果和最终结果均保留在 `experiments/results/external_nav/`，没有覆盖或混算：

| 尝试 | 结论 | 处理 |
|---|---|---|
| 默认安装目录的首次命令 | 启动前拒绝 | `xq_install` marker 不存在；未启动任何进程，随后显式绑定实际安装树 |
| `p4_cpu_a131a9e_smoke_20260917_1` | 启动前 ERROR | 记录实际安装目录不匹配，未进入仿真 |
| `p4_cpu_a131a9e_smoke_20260917_2` | FAIL | 运行门禁通过；P3 evaluator 清理竞态产生 traceback/process died |
| `p4_cpu_a131a9e_smoke_20260917_3` | PASS，中间组 | 验证最小退出修复；代码尚未形成冻结提交，不计最终组 |
| `p4_cpu_a131a9e_group01_run01` | FAIL | 错误 ArduPilot 依赖缺少 `GPS_TYPE`；未 ARM，终止确认完整 |
| `p4_cpu_a4d3fb3_smoke_20260917_1` | PASS | 最终冻结组冒烟 |
| `p4_cpu_a4d3fb3_group01_run01` | PASS | 最终冻结组 P4 冷启动 1 |
| `p4_cpu_a4d3fb3_group01_run02` | PASS | 最终冻结组 P4 冷启动 2 |
| `p4_cpu_a4d3fb3_group01_run03` | PASS | 最终冻结组 P4 冷启动 3 |

## 复现命令

以下命令在 `zyc111`、用户 `ld666`、仓库 `/home/ld666/projects/IMPACT` 中执行：

```bash
export ARDUPILOT_ROOT=/home/ld666/impact-deps/ardupilot
export ARDUPILOT_GAZEBO_ROOT=/home/ld666/impact-deps/ardupilot_gazebo

python3 scripts/impact.py test
python3 scripts/impact.py build

export IMPACT_BUILD_ROOT=/var/tmp/impact-1000-846fb8ab40
export IMPACT_INSTALL="$IMPACT_BUILD_ROOT/install"
export IMPACT_BUILD_MANIFEST="$IMPACT_BUILD_ROOT/build-manifest.json"
```

严格不解锁冒烟：

```bash
bash scripts/run_p4_external_nav.sh \
  --profile local_cpu \
  --smoke-only \
  --observation-seconds 30 \
  --run-dir experiments/results/external_nav/<new-smoke-run-id>
```

每次 P4 必须串行使用新的运行目录，并从无本任务残留进程和端口的冷状态启动：

```bash
bash scripts/run_p4_external_nav.sh \
  --profile local_cpu \
  --minimum-eval-duration 70 \
  --run-dir experiments/results/external_nav/<new-p4-run-id>
```

## 证据与限制

大型 bag 和 DataFlash 留在服务器。机器索引见
`evidence/P4_CPU_A4D3FB3_20260917/validation-index.json`，大型原件哈希见
`evidence/P4_CPU_A4D3FB3_20260917/LARGE_ARTIFACTS.sha256`。

- 三次通过只说明该冻结 CPU SITL 配置完成开发验收，不证明长期稳定性或算法收益。
- 本轮没有重新建立 GPU 三次验证组；历史 GPU/ExternalNav 告警仍作为独立问题保留。
- 本轮没有检查 Frontier 共享地图、EGO 碰撞地图或阶段 A 目标导航。
- Gazebo/ArduPilot 未提供统一的运行随机种子；已冻结可控的传感器桥种子和全部输入哈希。
- 报告/索引提交发生在飞行之后，属于纯文档提交；飞行 manifest 中的实际运行提交仍是
  `a4d3fb3`，不能改写为后续文档提交号。

下一阶段 A 的启动条件是以本冻结 CPU 基线建立新组，先运行
`normal / baseline / seed 1000` 固定目标导航，同时解析共享地图与 EGO 碰撞地图的实际内容、
坐标对齐、障碍膨胀和可达区域；正常到达、规划失败/超时和安全终止必须分别验收。阶段 A
不能替代旧 P5 探索验收，也不能以话题存在证明地图正确。
