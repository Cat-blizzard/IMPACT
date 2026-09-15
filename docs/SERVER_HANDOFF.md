# 给服务器执行方的交接单

更新：2026-09-16。仓库：https://github.com/Cat-blizzard/IMPACT ，分支：main。

## 1. 当前任务

在配有两张 PRO 6000 的服务器 WSL Ubuntu 22.04 中，先定位共享地图问题，再验证 P16 的 FAST-LIO → EGO → 完整性认证 → MAVROS → ArduPilot SITL → Gazebo 目标导航闭环，保存实际运行证据。总体目标仍是完整性感知自主探索，后续必须完成探索回接。

本机此前已有 8 个 ROS 包构建、160 项测试和真实 DDS/TF 接口测试通过；这不代表服务器全量构建、GPU 渲染或飞行已通过。当前没有正式 120 次结果或实际 demo 视频。

服务器最新报告（经用户转述、尚未由本机独立核验）：本地提交 `7c280014ccb53d3896cccd6b54b936f8c98d452d` 修复就绪检测，14 包构建、160 测试、GPU doctor 通过；P5 到达第一目标后因无法选出下一目标而停滞，最终未完成降落。保留该提交、stash、`p5_server_df0053d5` 运行目录和诊断包。

先读：

1. [开发顺序与验收边界](DEVELOPMENT_ROUTE.md)：本轮调整的优先约定
2. [安装与运行说明](IMPACT_LOCAL_SERVER.md)：区分现有命令与待实现门禁
3. [最新 demo 与基线约定](DEMO_AND_BASELINES.md)
4. `config/impact_v1.json` 和 `config/impact_dependencies.json`

## 2. 本次新讨论的结论

- 主视频改为双栏：**左 baseline，右 recovery（完整 IMPACT）**。
- 三栏 baseline / hard_gate / recovery 保留作补充分析。
- 正式实验仍是 normal / recoverable / unrecoverable 三场景、baseline / conservative / hard_gate / recovery 四策略、种子 0—9，共 120 次独立任务。
- 第五组“不确定性自适应减速”和外部论文算法暂未纳入，不自行扩展正式矩阵。
- 不预设基线失败或 IMPACT 获胜；完整记录等待、失败、超时及无收益结果。
- 现有 `render` 只能接收三个运行目录。双栏字幕、同步曲线和结果卡尚待适配，先采集数据，不把它们当作已实现入口。
- 旧 Frontier 可以替换，首版用预设目标验证共享闭环；恢复触发、动作选择和恢复成功必须由在线条件决定，不能编排航点或固定等待后自动放行。
- 不强制算法每次先停再动，允许在认证条件满足时继续或调整；提前调整能力不能在未实现/未验证时宣称具备。
- 单目标通过不能记成旧 P5 或自主探索通过；超时后成功降落也不能记成任务成功。
- 核心机制验证后必须回接在线目标生成、完整性参与目标选择、恢复续接和探索终止，按 C1—C4 单独验收。

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

**已有服务器本地提交 `7c2800…` 时：** 它与后续文档提交可能使 main 分叉，此时 `pull --ff-only` 会拒绝执行。先保留提交和 stash，在工作区干净时为该提交建立备份分支（已有则无需重复创建），然后 fetch 并合并 origin/main：

```bash
git branch backup/p5-startup-7c280014 7c280014ccb53d3896cccd6b54b936f8c98d452d
git fetch origin
git merge --no-edit origin/main
git rev-parse HEAD
```

出现冲突时保留双方改动并审阅，不丢弃启动修复或 stash。合并后记录版本，将代码修复正常推送回同一仓库，供两端审阅同步。本次文档提交不包含服务器的修复代码。

## 4. 当前下一步：同步修复、地图排查、新门禁开发

按以下顺序推进：

1. 同步并审阅服务器启动修复。核查新 P16 运行脚本中同类 ROS CLI 就绪探测/诊断调用，迁移适用的探针修复，不能只修旧 P5。
2. 在第一目标前后检查 Frontier 二维投影与 EGO 三维碰撞地图、起点及邻域、时间/坐标对齐，回答可达性异常是否污染新链路。
3. 若只是旧 Frontier 选点/完成判据问题，可停止以旧探索完全通过为唯一目标；共享地图有错则先修复。
4. 实现独立命名、有协议版本的基础目标导航门禁，验证正常到达与失败后的终止确认。保留旧 P5 FAIL 记录。
5. 更新 validate/batch 对新协议的检查并补相应测试。新协议通过后再进入四策略试跑和正式矩阵；不通过手写 PASS 或删除门禁达成迁移。

完整要求和探索回接里程碑见 DEVELOPMENT_ROUTE.md。新流程目前尚未实现，没有可直接替代旧门禁的新命令。

### 环境和构建命令

在同一 WSL 终端设置：

```bash
export ARDUPILOT_ROOT="$HOME/impact-deps/ardupilot"
export ARDUPILOT_GAZEBO_ROOT="$HOME/impact-deps/ardupilot_gazebo"
export IMPACT_BUILD_ROOT="$HOME/impact-build"
export IMPACT_BUILD_JOBS=2
mkdir -p "$HOME/impact-deps"
```

ROS 2 Humble 和外部依赖未装好时，按安装说明执行 `bash scripts/impact_server_install.sh`。该脚本需要 sudo；现有外部仓库与锁定版本不符时使用专用目录，不覆盖其他工程。

代码修复后按需重新构建、测试、检查环境；已有健康的环境不必反复重装：

```bash
bash scripts/impact.sh build
bash scripts/impact.sh test
bash scripts/impact.sh doctor --profile server_gpu --runtime
```

当前 `validate-server` 仍自动执行旧 P5，再执行普通场景与四策略试跑，会继续被旧探索卡住。它可以保留作原版诊断/复现入口，但不是新门禁已完成的证明。阶段 A 的名称、产物和 batch 依赖须随实现一同更新文档。

GPU 检查应包括 doctor 的 GL 输出和实际 Gazebo 进程的 runtime-audit；单看 nvidia-smi 能列出显卡不够。检查失败时回传日志，不关闭检查或改软件渲染后报告 GPU 成绩。

默认串行，不启用双任务或 `--jobs 2`。两张同型号 GPU 的渲染绑定与通信隔离尚未验收。时延数据在受控负载下单独解释。

## 5. 下一轮需要回传什么？

简要报告：

```text
提交 ID：
Ubuntu / ROS / Gazebo / ArduPilot / plugin 版本：
GPU 型号、驱动、GL renderer、runtime-audit 结果：
全量构建及测试结果：
原 P5 结果和目录：
共享地图异常属于哪一层、是否影响 EGO 碰撞地图、证据目录：
新基础导航门禁是否实现、协议名称/版本及测试：
普通场景结果和目录：
失败任务是否完成降落/解除解锁，任务结果与终止结果是否分开：
退化四策略结果和目录：
在线行为及其依据（继续/调整/制动/恢复/退出，是否有新观测与重新认证）：
当前失败命令、退出码、关键错误：
诊断包和完整记录位置：
```

保留所有运行目录，重点包括 run.json、doctor.json、mission.json、evaluation.json、events.jsonl、telemetry.jsonl、performance.json、runtime-audit.json、进程日志、版本/配置哈希、rosbag 和 gz_record。

失败通常已自动生成诊断包；补打包用：

```bash
bash scripts/impact.sh bundle <实际运行目录>
```

先回传小诊断包和报告。rosbag、视频等大文件按 artifact-inventory.json 的路径、大小和哈希另行保留，必要时再传。不要删掉失败结果，也不要修改 JSON 状态为 PASS。

## 6. 新协议通过后再运行正式矩阵

先核查三场景是否实际呈现预期的观测条件，检查制动与跟踪预算，并冻结参数。需要修复或调参时使用开发种子，不以正式种子反复调试。

正式规模仍为 120 次目标导航任务。当前 batch 的 `--validation` 接收旧 server-validation.json；新目标导航门禁的 schema、状态和源码/配置/依赖核验尚待实现。实现后在本节补充验证过的命令和产物路径，不能把旧 P5 标记改名后复用。

续跑仍须保留完整任务失败，不能重跑到成功。代码、配置或外部二进制变化后需重新构建/阶段验证，并使用新的正式结果目录。目标导航结果不计作自主探索结果；后续 C1—C4 使用独立探索协议。

## 7. 为视频保留的数据

优先寻找同场景、同种子的 baseline 与 recovery 记录，说明选取理由，同时保留四组完整统计。

两侧从各自任务 ACTIVE 时刻对齐，统一视角、播放倍率和时间轴；先结束的一侧应标记结束。状态字幕与 PL/AL/Margin 来源于日志，真值误差只用于评估。保留原始记录，不仅回传剪辑视频。

如果没有真实恢复收益，按实际结果报告并分析，不制造基线失败或虚构恢复事件。展示规格和待开发项以 DEMO_AND_BASELINES.md 为准。
