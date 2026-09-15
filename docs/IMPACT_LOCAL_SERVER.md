# IMPACT 本机开发与服务器仿真交付

更新：2026-09-16。新增闭环称为 P16，原 P1—P15 入口和 evidence 保留。

最新展示约定见 [Demo 与基线补充方案](DEMO_AND_BASELINES.md)，可直接交给服务器执行方的说明见 [服务器交接单](SERVER_HANDOFF.md)。主视频改为左基线、右完整 IMPACT 的双栏；正式四策略、120 次矩阵保持不变。现有渲染器仍为三栏，双栏展示适配尚待完成。

## 1. 当前交付状态

已实现统一入口、EGO 候选轨迹关联、最终轨迹认证、唯一 MAVROS 位置出口、制动目标、恢复临时目标管理、独立真值评估、场景生成、配对种子批处理、诊断包和三栏回放渲染器。

本机已完成 8 个 ROS 包构建（含新消息和 EGO C++）、160 项回归测试及真实 DDS/TF 指令接口测试。CPU 构建只覆盖 `xq_autonomy`、`ego_planner` 及依赖；Gazebo bridge 和完整 FAST-LIO 仿真组合需要服务器全量构建。本机没有 `gz`，因此没有飞行闭环验收、正式统计结果或真实演示视频。安装脚本、渲染器和新场景均须服务器验证。

**“本机代码与接口通过”和“服务器闭环通过”是两个状态。** 场景名称 recoverable/unrecoverable 是待检验假设；当前未证明哪组策略会获益。GPU 渲染审计使用 GL 信息和 Gazebo 进程驱动映射，若 WSL 驱动布局不被识别会失败，需带日志调整，不能关闭检查后当作 GPU 成绩。

## 2. 工程结构与控制边界

```text
FAST-LIO → 注册点云 / 定位 / 当前完整性协方差
                    ↓
任务/恢复目标 → EGO → PlannerCandidate → 在线最终认证
                                      ↓
                          授权消息 + certified_bspline
                                      ↓
                          traj_server → 指令仲裁器 → MAVROS
                                      ↓
                         ArduPilot SITL ↔ Gazebo

Gazebo 真值 → 独立评估器 / rosbag（不接入规划和认证）
```

- `config/impact_v1.json`：冻结场景、策略、种子、目标、任务超时、传感噪声和校准位置。
- `config/impact_dependencies.json`：外部源码版本及本机消息依赖回退版本。
- `src/xq_autonomy/xq_autonomy/sitl_*.py`：认证、仲裁、恢复、任务和评估。
- `src/xq_sim_bringup/launch/impact_sitl.launch.py`：完整新闭环。
- `scripts/impact.sh`：Linux/WSL 统一入口；不提交或推送 Git。

P10 只提出有限恢复意图；真正执行的是 EGO 优化后、按当前协方差重新认证的轨迹。预测协方差只用于排序。短段执行后须取得新观测，并再次放行任务轨迹才记录 `RECOVERY_CONFIRMED`。

所有位置指令只由新仲裁器发出。起飞和降落由任务管理器请求 FCU 模式/命令，仲裁器在这些阶段停止竞争位置控制。拒绝新轨迹时旧轨迹只有在授权仍有效时可继续。授权过期、定位/地图过期或跟踪偏差过大触发有停止距离的制动目标。这里的减速度与跟踪预算必须通过服务器实际运动核验；通过软件测试不代表动力学保证已经成立。

时间：任务/数据关联使用 ROS 仿真时间；看门狗用 steady timer 和 monotonic；认证计算耗时用 perf_counter。原始传感器、定位和命令时间戳随 rosbag 保留。仿真重置锁定控制授权，必须新建任务会话再运行。

坐标：本轮使用 upright ENU 的 `xq_lio_map → map` 明确静态约定，与 P4 ExternalNav 对齐；仲裁器实际应用 TF 平移/旋转并变换航向。倾斜地图、错误 frame、非有限坐标被拒绝。此约定只适用于当前固定初始化方案，不能把不同坐标系仅改名后接入。

## 3. 本机日常开发

Windows 源码为 `D:\IMPACT`。在 WSL Ubuntu 22.04 内：

```bash
cd /mnt/d/IMPACT
bash scripts/impact.sh doctor --profile local_cpu
bash scripts/impact.sh build --cpu-only
bash scripts/impact.sh test
```

默认将源代码复制到 `/var/tmp/impact-<uid>-<路径哈希>/source/src`，build/install/log 均留在 Linux 文件系统。可设置 `IMPACT_BUILD_ROOT` 指向专用 Linux 目录；禁止 Windows 挂载目录作为构建根。构建期间修改源码会使构建记录失效，需重跑。

`test --pure` 只运行新增 ROS 无关测试。完整 `test` 还运行既有回归和实际 ROS 消息链路测试。需要 Humble 的 `mavros_msgs`、`geographic_msgs`。本机 apt 的 ROS 源连接失败，已用依赖清单中的官方源码提交构建这两个消息包；这不等同于安装完整 MAVROS。服务器优先安装发行版 MAVROS。

有 Gazebo 后可选 CPU 小场景：

```bash
bash scripts/impact.sh build
bash scripts/impact.sh run --profile local_cpu --scenario normal --strategy baseline --seed 1000 --results experiments/results/cpu_smoke
```

`local_cpu` 无 GUI、软件渲染，不计入服务器性能成绩。缺依赖时记录 ERROR 和诊断包，不生成假成功结果。

## 4. 向服务器传代码

本轮代码通过 [Cat-blizzard/IMPACT](https://github.com/Cat-blizzard/IMPACT) 的 `main` 分支同步。在服务器 WSL 的 Linux 文件系统中首次拉取：

```bash
mkdir -p "$HOME/work"
cd "$HOME/work"
git clone https://github.com/Cat-blizzard/IMPACT.git
cd IMPACT
git rev-parse HEAD
```

已从该仓库 clone 的服务器工程，后续在工作区干净时运行 `git pull --ff-only origin main`，再重新构建。开始正式实验时记录提交 ID，批次运行期间不要更新代码。

先前提供的源码 ZIP 仍可用作该次本机交付的备份；包中 `source-snapshot.json` 逐文件列出 SHA-256，ZIP 旁有总哈希。源码包不含 `.git` 和本机 install 树，不能直接在 ZIP 解压目录中执行 `git pull`；需要 Git 同步时使用上面的 clone 目录。

本机后续重新导出：

```bash
python3 scripts/impact.py package --output /mnt/d/IMPACT-source.zip
```

两端各自构建，不同步 install 树。原始项目 [Accelerate11/IMPACT](https://github.com/Accelerate11/IMPACT) 保留为本机 `upstream`，日常推送目标 `origin` 为 `Cat-blizzard/IMPACT`。不要向旧安装树直接覆盖单个 Python 文件后继续正式实验。

## 5. 服务器安装（WSL Ubuntu 22.04）

Windows 主机先确认 PRO 6000 驱动与 WSLg 工作正常；WSL 支持的 OpenGL 加速来自主机驱动，参见 [Microsoft WSL GUI 文档](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)。WSL `nvidia-smi` 能看到 GPU 不代表 Gazebo 已使用 GPU。

先按 [ROS Humble Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 安装 Humble。然后在 WSL 工程目录：

```bash
export ARDUPILOT_ROOT="$HOME/impact-deps/ardupilot"
export ARDUPILOT_GAZEBO_ROOT="$HOME/impact-deps/ardupilot_gazebo"
export IMPACT_BUILD_ROOT="$HOME/impact-build"
export IMPACT_BUILD_JOBS=2
mkdir -p "$HOME/impact-deps"
bash scripts/impact_server_install.sh
bash scripts/impact.sh build
bash scripts/impact.sh test
bash scripts/impact.sh doctor --profile server_gpu --runtime
```

环境变量每个新终端都需设置。安装脚本需 sudo，会安装系统依赖、Harmonic、MAVROS/地理数据并构建独立 ArduPilot/plugin。已有外部仓库不是干净的目标版本时会拒绝覆盖，改用专用新路径。官方依据：[Gazebo Harmonic 安装](https://gazebosim.org/docs/harmonic/install_ubuntu/)、[ArduPilot Gazebo 集成](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)。

ArduPilot 使用原 P5 报告的 Copter-4.5.7；plugin 的具体提交固定在依赖清单，当前尚未在此机器编译验证。apt 包记录实际版本到构建/运行产物，正式实验前冻结服务器环境，期间不升级依赖。

`doctor --runtime` 必须通过；`server_gpu` 禁止 llvmpipe/softpipe。运行时会再审查 Gazebo 本身的驱动映射，并保存 GPU 使用日志。默认串行；`--jobs 2` 明确拒绝，因为双任务 GPU 绑定、通信和端口隔离尚未在该服务器验证。两张同型号卡不能仅凭 `CUDA_VISIBLE_DEVICES` 就宣称完成 Gazebo 渲染绑定。

## 6. 按顺序试跑，再执行正式矩阵

```bash
# 原 P5 → 普通场景 baseline/recovery → 退化场景四策略（开发种子 1000）
bash scripts/impact.sh validate-server --profile server_gpu --results experiments/results/impact_v1

# 阶段检查通过后：3 × 4 × 10 = 120 次任务
bash scripts/impact.sh batch --profile server_gpu --jobs 1 \
  --validation experiments/results/impact_v1/server-validation.json \
  --results experiments/results/formal_v1
```

`validate-server` 在原 P5 或普通任务未通过时停止。退化试跑中“确认终止的任务失败”是有效结果；若飞控/通信/评估缺失导致无法确认则停止。试跑通过只表示进入正式测量的工程条件具备，不保证恢复收益。

相同 `batch` 命令可断点续跑：已经完成的失败任务也跳过，不重跑到成功为止。基础设施失败先检查诊断包，修复后重跑。源码、配置或外部二进制变化会使冻结协议失效；重新 build 和 validate-server，并使用新的正式结果目录。正式种子 0—9，开发种子 1000—1004，避免把调参试跑混入正式结果。

单次调试：

```bash
bash scripts/impact.sh run --profile server_gpu --scenario recoverable --strategy recovery --seed 1001 --results experiments/results/dev
```

三种场景由真实世界几何生成：normal 增加观测结构，recoverable 在局部放置恢复机会，unrecoverable 为长平行通道。种子实际驱动 bridge 传感噪声 RNG 和特征几何扰动；同场景/种子各策略共用世界，但各自沿真实轨迹采集观测。仍需确认三场景在服务器呈现预期定位退化，禁止为讲故事预设策略输赢。

## 7. 结果、指标与回传

每次尝试使用独立目录，包含：

| 文件 | 含义 |
|---|---|
| run.json / config.json / build-manifest.json | 会话、源码、参数、种子、最终状态 |
| calibration.json / world.sdf / scenario.json | 实際运行的校准、世界和独立几何 |
| mission.json | 任务成功与终止确认分开，含失败原因 |
| evaluation.json / telemetry.jsonl | 真值净空、定位误差、轨迹、逐时刻 AL/PL/Margin |
| events.jsonl / performance.json | 请求、认证、撤销、恢复、新观测、认证耗时与恢复增益 |
| runtime-audit.json / *-graph.txt | 真值隔离、唯一指令出口、渲染驱动检查 |
| rosbag/ / gz_record/ | 原始 ROS 消息与 Gazebo 未剪辑记录 |
| *.log / system-packages.txt / external-binaries.sha256 | 进程和环境证据 |

碰撞按独立世界 OBB 与 0.35 m 机体包络的几何接触计，字段注明该定义；不是另一个 Gazebo contact-sensor 真值标签。定位 ATE 只做首次姿态/位置固定对齐；PL 覆盖比较误差在当前关键方向上的投影，不把 3D 误差范数直接与方向 PL 比较。帧匹配容差 0.05 s，丢失匹配单独计数。停止时间同时报告低速累计时长和撤销到停止时长。

`RECOVERY_CONFIRMED` 拆分 ΔM = ΔAL − ΔPL；前后轨迹可能有不同关键点和方向，不能把全部裕度变化归因于定位改善。`performance.json` 目前是认证模块真实计算耗时分位数、超过 100 ms 的次数和观测区间实时率，**不是整条传感器到飞控链路的端到端时延**。正式时延测量应在串行、受控负载下进行。

汇总命令：

```bash
bash scripts/impact.sh summarize experiments/results/formal_v1
bash scripts/impact.sh bundle experiments/results/dev/<某次运行目录>
```

`summary.json` 保留全部尝试和基础设施重试；`tasks.csv` 和 `task-statistics.json` 每个场景/策略/种子取首个完整任务，保留失败，统计单位为任务。未自动生成逐帧显著性检验。样本尚未完整时不能以部分成功任务代表正式 120 次矩阵。

失败自动生成 `*-diagnostics.tar.gz`；超过 20 MB 的单个文件和 rosbag/video 不塞进小包，`artifact-inventory.json` 记录完整文件大小与哈希。先回传诊断包，必要时再传完整 rosbag。磁盘使用会随点云记录增长，先以试跑的实际大小估计 120 次任务存储预算。

## 8. 回放与视频

```bash
bash scripts/impact.sh replay <运行目录> --kind gazebo
bash scripts/impact.sh replay <运行目录> --kind rosbag
```

回放采用 ROS domain 168、独立 Gazebo partition，不启动飞控。ROS 回放在另一终端可用相同 domain 打开 RViz；播放原始 bag 中的 /clock，不再生成第二个时钟。Gazebo 回放支持情况在服务器验证后确认。

主视频采用左 baseline、右 recovery 的双栏，布局、同步、字幕、结果卡和公平比较要求见 [补充方案](DEMO_AND_BASELINES.md)。当前代码尚未支持双栏，不能直接把两个目录传给 render。

现有三栏渲染入口作为补充，选同场景、同种子、相同源码和校准的三种策略，例如 baseline / hard_gate / recovery：

```bash
bash scripts/impact.sh render <baseline目录> <hard_gate目录> <recovery目录> --output recovery-comparison.mp4
```

需要 matplotlib 和 ffmpeg。渲染器读取真实 telemetry，同步绘制三维场景、飞行轨迹与 AL/PL/Margin，并标注软件在环/真值仅评估。它是记录状态重建的可视化，不是虚构飞行或直接录屏。保留完整回放，再从有完整因果链的运行编辑约三分钟演示；没有观察到恢复收益就报告实际结果，不制作相反结论。

## 9. 常见问题

| 现象 | 下一步 |
|---|---|
| gz 不存在 | 完成 Harmonic 安装；此时本机 CPU 接口测试仍可做 |
| GPU 可见但 hardware_gl 失败 | 查看 glxinfo、WSLg display 和主机驱动；不要改成 local_cpu 后计为 GPU |
| runtime-audit 失败 | 查看 Gazebo 日志/驱动映射与 topic graph，回传诊断包 |
| 5760 / 9002 冲突 | 停止自己确认不用的旧任务；脚本不会杀别人的进程 |
| 长期无认证通过 | 查看 CERTIFY reason、输入年龄、跟踪误差、AL/PL；先验证 frame 和模型预算 |
| 无可行恢复候选 | 允许制动等待，超时算失败；检查新场景是否实际提供观测收益 |
| 没有 mission/evaluation 结果 | 基础设施错误，批次暂停；不要手写 PASS |
| LANDING_NOT_CONFIRMED | 终止未通过，不能算完整任务，回传飞控和任务日志 |
| 修改后 full build missing/stale | 重新完整 build；正式批次需新验证和新结果目录 |

## 10. 服务器待验收清单

- 全量 colcon 构建和外部版本组合；实际 PRO 6000 渲染。
- 原 P5、新普通任务的起飞、规划、执行、降落闭环。
- 制动距离/跟踪误差预算实测；传感器失效、恢复目标和迟到轨迹在完整闭环中的行为。
- recoverable 场景是否有真实恢复收益；unrecoverable 是否合理停止。
- 120 次独立任务、失败结果保留、统计表/图与完整录制。
- 现有三栏渲染验证；双栏主视频适配及约三分钟演示最终剪辑。

双 GPU 并行和完整端到端时延测量尚未实现服务器验收，不作为此次本机交付已完成项。预核查静态场景的仿真降落不构成任意环境安全着陆保证。
