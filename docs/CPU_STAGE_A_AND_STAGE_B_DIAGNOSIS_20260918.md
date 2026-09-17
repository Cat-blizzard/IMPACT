# CPU Stage A 验收与 Stage B 诊断（2026-09-18）

## 结论

- CPU 软件渲染下的独立无解锁冒烟通过。
- 固定目标的 Stage A `normal/baseline/seed=1000` 通过：真实定位、注册点云、EGO、飞控执行、目标到达、LAND/解除武装和清理证据完整。
- `normal/recovery/seed=1000` 也完成了目标任务，但全程没有恢复触发，只能证明该策略在本次正常任务中没有产生误触发，不能证明恢复有效。
- Stage B 尚未通过。`recoverable/recovery/seed=1000` 虽到达目标，却没有任何在线恢复触发；`unrecoverable/recovery/seed=1000` 在约 x=3.28 m 停滞并超时，但 175 次轨迹认证全部接受，仍不是恢复门禁触发。
- 暂停扩展三场景、四策略和正式矩阵。下一步应先修正场景/规划链路，使开发样本真实产生预期完整性条件，再按新的机制门禁验证。

这些是开发验收结果，不是稳定性统计或恢复收益证明。旧 P5 FAIL、1.2 m `UNVERIFIED_FLIGHT`、GPU P4 未通过和 ExternalNav/EKF 历史异常继续保留。

## 冻结环境

- 飞行提交：`75f4e9652709825275237ae2f8b5143f8466a18d`
- 源码哈希：`c2e95b658bcfbe1bd4ea64aef84110aea49813cca6a28a6f4d78175ee1cfd189`
- 构建清单哈希：`f752f86e43d41207da7e42e8ffc8c698695a034c9075d6e8f92a6191bfad5e1d`
- 安装目录：`/var/tmp/impact-1000-846fb8ab40/install`
- 配置哈希：`ced0250c794f95e748179261c59de8f2e4403590ed927ccae529a045880c37a6`
- 标定哈希：`771bdffcf3d4422d4641424dab326a08aa5be2b0dffd7f9d2f2f9ff82ea9f038`
- 场景种子：`1000`
- 配置：`local_cpu`
- 渲染器：Mesa llvmpipe LLVM 15.0.7，软件渲染；没有标为 GPU 结果。

后续审计代码提交增加机制门禁并纠正事件语义，没有修改上述历史运行记录。该提交在合并前完成完整测试和重建，但没有冒充 `75f4e96` 的飞行提交。

后续代码构建源码哈希为 `a1563bdfa049b04afee413a04b979d5013fa09853f9bb83eaa84ee6351f218ae`，构建清单哈希为 `fc04beaabb64b44c491148d229acc884185e001ad8df0f8f8827711c884c32ee`；14 个包构建通过，安装树中的 supervisor 与源码哈希一致。

## 有效验收

### 无解锁冒烟

运行目录：

`experiments/results/stage_a_cpu_75f4e96/smoke/normal-recovery-s1000-20260918T035955-e9e5c7cf`

- 固定观测窗口约 30.02 s。
- `/localization/odom` 220 条；ExternalNav 状态 150 条，全部健康；FCU 状态 21 条，武装样本 0。
- ExternalNav 发布端唯一，MAVROS 订阅端存在，未混入旧实例。
- mission 节点不存在，没有 ARM/TAKEOFF。
- bag 可读取，进程清理通过，无 Gazebo 段错误。

### Stage A baseline

运行目录：

`experiments/results/stage_a_cpu_75f4e96/normal-baseline-s1000/normal-baseline-s1000-20260918T040142-5d0f575a`

- `GOAL_REACHED`，任务成功，终止通过新鲜 FCU 状态确认 LAND 且解除武装。
- 291 个评估样本，0 次碰撞，路径 12.423 m，ATE RMS 0.0872 m，最小真值净空 1.289 m。
- PL 覆盖率 1.0，可用率 0.9965。
- 注册点云、EGO 膨胀图与 Frontier 图均有实际内容；时间对齐与固定目标可达性检查通过。
- 授权行为审计和运行时契约通过；退出清理通过。

### Stage A recovery 策略正常任务

运行目录：

`experiments/results/stage_a_cpu_75f4e96/normal-recovery-s1000/normal-recovery-s1000-20260918T040439-b018a7ac`

- `GOAL_REACHED`，终止确认完整。
- 357 个评估样本，0 次碰撞，路径 12.763 m，ATE RMS 0.0637 m，最小真值净空 1.258 m。
- 0 个 `RECOVERY_FORECAST`、0 个恢复步骤、0 个恢复收益确认。
- 判定：Stage A 正常任务通过；恢复机制 `NOT_DEMONSTRATED`。

## 保留的失败与诊断

### 旧恢复状态机失败

`d47344e` 的 `normal/recovery/seed=1000` 在 180 s 超时，安全降落并解除武装。记录中有 12 个预测周期、35 次恢复轨迹授权和 11 次新观测，但没有 `RECOVERY_STEP_DONE`；所有观测都由短悬停闭合。由此修复了轨迹结束后未给飞行器减速到达的等待缺口，形成 `75f4e96`。

重新用严格机制门禁审计后，该旧样本为 `NOT_DEMONSTRATED`。历史 `RECOVERY_CONFIRMED` 名称不再被当作恢复通过证据。

### recoverable 场景没有产生恢复机会

运行目录：

`experiments/results/stage_b_cpu_75f4e96/recoverable-recovery-s1000/recoverable-recovery-s1000-20260918T040813-15d2f466`

- 任务到达、0 碰撞、终止和清理通过。
- 0 个预测周期、0 个恢复授权、0 个恢复步骤、0 个收益确认。
- 认证余量比 normal 更大：移除部分特征柱后，PL 中位数约 0.066 m，AL 中位数约 0.555 m，余量中位数约 0.479 m。
- 当前 `recoverable` 名称只是未验证假设，不能用于宣称恢复收益。

### unrecoverable 场景是规划停滞，不是门禁拒绝

运行目录：

`experiments/results/stage_b_cpu_75f4e96/unrecoverable-recovery-s1000/unrecoverable-recovery-s1000-20260918T041412-94a6f526`

- 任务 `TASK_TIMEOUT`，实际在约 x=3.28 m 停滞约 159.1 仿真秒；结果保留为 FAIL。
- 175 次轨迹认证全部接受；PL 0.0517--0.1061 m，AL 0.4783--0.7063 m，最小余量 0.4185 m。
- 0 个预测周期、0 个恢复步骤。EGO 持续生成局部轨迹，问题应从规划推进/执行链路继续定位，不能归因为完整性恢复失败。
- 任务失败与终止分开记录：最终新鲜 FCU 状态确认 LAND、解除武装；0 碰撞；bag 可读取；清理通过。

## 新机制门禁

`scripts/analyze_recovery_causality.py --require-mechanism-pass` 现在要求以下证据全部成立：

1. 在线判定实际触发恢复；
2. 至少一条恢复轨迹经过最终在线认证并获授权；
3. 飞行器实际完成恢复步骤；
4. 完成步骤后取得新观测；
5. 新观测后任务轨迹重新获授权；
6. 实测余量相对恢复前增加。

审计运行完成仍记为 `DIAGNOSTIC_COMPLETE`；机制未满足则另记为 `NOT_DEMONSTRATED`。服务器开发验收在 `recoverable/recovery` 上强制检查该门禁，不能再凭任务文件存在或单次到达继续正式批次。

运行时事件也已纠正：只有 `delta_margin > 0` 才记录 `RECOVERY_CONFIRMED`，否则记录 `RECOVERY_NOT_BENEFICIAL`。这项修改只改变证据语义，不改变飞行控制决策。

## 复现与下一步

```bash
export ARDUPILOT_ROOT=/home/ld666/impact-deps/ardupilot
export ARDUPILOT_GAZEBO_ROOT=/home/ld666/impact-deps/ardupilot_gazebo
bash scripts/impact.sh build
bash scripts/impact_test.sh "$PWD" /var/tmp/impact-1000-846fb8ab40

python3 scripts/analyze_recovery_causality.py RUN_DIR \
  --estimator-memory-horizon-s 3.0 \
  --require-mechanism-pass
```

下一轮只处理一个因素：先解释 `unrecoverable` 中 EGO 在 x≈3.28 m 持续重规划却不推进的原因，并重新设计或修正能真实产生可恢复完整性条件的场景。形成新冻结版本后，只运行一个 `recoverable/recovery/seed=1000` 开发样本。门禁通过前不扩展组合、不启动正式矩阵，也不以重复试飞挑选成功样本。
