# 给服务器执行方的交接单

更新：2026-09-16。仓库：https://github.com/Cat-blizzard/IMPACT ，分支：main。

## 1. 当前任务

在配有两张 PRO 6000 的服务器 WSL Ubuntu 22.04 中，验证 P16 的 FAST-LIO → EGO → 完整性认证 → MAVROS → ArduPilot SITL → Gazebo 闭环，保存实际运行证据。

本机此前已有 8 个 ROS 包构建、160 项测试和真实 DDS/TF 接口测试通过；这不代表服务器全量构建、GPU 渲染或飞行已通过。当前没有正式 120 次结果或实际 demo 视频。

先读：

1. [安装与运行说明](IMPACT_LOCAL_SERVER.md)
2. [最新 demo 与基线约定](DEMO_AND_BASELINES.md)
3. `config/impact_v1.json` 和 `config/impact_dependencies.json`

## 2. 本次新讨论的结论

- 主视频改为双栏：**左 baseline，右 recovery（完整 IMPACT）**。
- 三栏 baseline / hard_gate / recovery 保留作补充分析。
- 正式实验仍是 normal / recoverable / unrecoverable 三场景、baseline / conservative / hard_gate / recovery 四策略、种子 0—9，共 120 次独立任务。
- 第五组“不确定性自适应减速”和外部论文算法暂未纳入，不自行扩展正式矩阵。
- 不预设基线失败或 IMPACT 获胜；完整记录等待、失败、超时及无收益结果。
- 现有 `render` 只能接收三个运行目录。双栏字幕、同步曲线和结果卡尚待适配，先采集数据，不把它们当作已实现入口。

## 3. 拉取代码，记录版本

首次在 WSL 的 Linux 文件系统中：

```bash
mkdir -p "$HOME/work"
cd "$HOME/work"
git clone https://github.com/Cat-blizzard/IMPACT.git
cd IMPACT
git rev-parse HEAD
```

已有该仓库时，在工程目录先看 `git status --short` 和 `git remote -v`。工作区干净且 origin 是 Cat-blizzard/IMPACT 时：

```bash
git pull --ff-only origin main
git rev-parse HEAD
```

有本地改动或分支分叉时先记录并处理，不使用强制 reset、强推或覆盖旧结果。记录实际提交 ID；正式批次开始后冻结代码和环境。

## 4. 第一轮：只做环境检查、构建和阶段试跑

在同一 WSL 终端设置：

```bash
export ARDUPILOT_ROOT="$HOME/impact-deps/ardupilot"
export ARDUPILOT_GAZEBO_ROOT="$HOME/impact-deps/ardupilot_gazebo"
export IMPACT_BUILD_ROOT="$HOME/impact-build"
export IMPACT_BUILD_JOBS=2
mkdir -p "$HOME/impact-deps"
```

ROS 2 Humble 和外部依赖未装好时，按安装说明执行 `bash scripts/impact_server_install.sh`。该脚本需要 sudo；现有外部仓库与锁定版本不符时使用专用目录，不覆盖其他工程。

依次运行，前一步失败则停止：

```bash
bash scripts/impact.sh build
bash scripts/impact.sh test
bash scripts/impact.sh doctor --profile server_gpu --runtime
bash scripts/impact.sh validate-server --profile server_gpu --results experiments/results/impact_v1
```

`validate-server` 自动执行原 P5、普通场景 baseline/recovery、退化场景四策略开发试跑，使用开发种子 1000。普通场景失败或基础设施/终止确认失败时停止。退化场景中“有完整评估且确认终止的任务失败”是有效结果，不等于基础设施坏了。

GPU 检查应包括 doctor 的 GL 输出和实际 Gazebo 进程的 runtime-audit；单看 nvidia-smi 能列出显卡不够。检查失败时回传日志，不关闭检查或改软件渲染后报告 GPU 成绩。

默认串行，不启用双任务或 `--jobs 2`。两张同型号 GPU 的渲染绑定与通信隔离尚未验收。时延数据在受控负载下单独解释。

## 5. 第一轮需要回传什么？

简要报告：

```text
提交 ID：
Ubuntu / ROS / Gazebo / ArduPilot / plugin 版本：
GPU 型号、驱动、GL renderer、runtime-audit 结果：
全量构建及测试结果：
原 P5 结果和目录：
普通场景结果和目录：
退化四策略结果和目录：
是否观察到“撤销→制动→恢复动作→新观测→重新认证→继续任务”：
当前失败命令、退出码、关键错误：
诊断包和完整记录位置：
```

保留所有运行目录，重点包括 run.json、doctor.json、mission.json、evaluation.json、events.jsonl、telemetry.jsonl、performance.json、runtime-audit.json、进程日志、版本/配置哈希、rosbag 和 gz_record。

失败通常已自动生成诊断包；补打包用：

```bash
bash scripts/impact.sh bundle <实际运行目录>
```

先回传小诊断包和报告。rosbag、视频等大文件按 artifact-inventory.json 的路径、大小和哈希另行保留，必要时再传。不要删掉失败结果，也不要修改 JSON 状态为 PASS。

## 6. 阶段通过后再运行正式矩阵

先核查三场景是否实际呈现预期的观测条件，检查制动与跟踪预算，并冻结参数。需要修复或调参时使用开发种子，不以正式种子反复调试。

```bash
bash scripts/impact.sh batch --profile server_gpu --jobs 1 \
  --validation experiments/results/impact_v1/server-validation.json \
  --results experiments/results/formal_v1
```

同一命令可续跑；完整任务失败也应保留并跳过，不能重跑到成功。代码、配置或外部二进制变化后需重新构建/阶段验证，并使用新的正式结果目录。

## 7. 为视频保留的数据

优先寻找同场景、同种子的 baseline 与 recovery 记录，说明选取理由，同时保留四组完整统计。

两侧从各自任务 ACTIVE 时刻对齐，统一视角、播放倍率和时间轴；先结束的一侧应标记结束。状态字幕与 PL/AL/Margin 来源于日志，真值误差只用于评估。保留原始记录，不仅回传剪辑视频。

如果没有真实恢复收益，按实际结果报告并分析，不制造基线失败或虚构恢复事件。展示规格和待开发项以 DEMO_AND_BASELINES.md 为准。
