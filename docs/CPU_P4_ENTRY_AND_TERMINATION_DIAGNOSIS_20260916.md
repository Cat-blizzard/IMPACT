# CPU P4 入口与终止路径诊断（2026-09-16）

## 当前结论

本轮继续暂停飞行重试，不把新的失败样本改写为 P4 验证组结果。当前代码修复和回归测试已完成；安装树已经重新构建，但修复后的专用 P4 尚未重新飞行，因此不能宣称新增 P4 PASS。

## 运行入口核对

历史有效 P4 样本使用：

```text
scripts/run_p4_external_nav.sh
  -> xq_p4_external_nav.launch.py
  -> xq_p4_mission
```

最近的 `cpu_p4_prearm_gate_validation/.../8d992f42` 使用的是 `scripts/impact.py run`：

```text
scripts/impact_run.sh
  -> impact_sitl.launch.py
  -> impact_mission
```

两者不是同一运行配置。后者还启动 Frontier、完整性、EGO、仲裁器和通用任务节点，结果文件为 `mission.json` schema 1；专用 P4 使用 `mission-result.json` schema 2。该失败样本保留为通用闭环的失败诊断，不能与历史专用 P4 PASS 混成同一验证组。

当前安装树绑定结果：

- Git HEAD：`95f0fc51b7da8288cee6d9ee3e4bf7f44bef51f7`
- 安装目录：`/home/ld666/impact-build/install`
- 源码哈希：`1cf8f90b82c58e160cb82240ec494276996c2596d081d0fa4cdc5c03aa55004d`
- 安装清单哈希：`a1c4173dc725bfd6f55dfa8cb33622e0a8a1ce56bc37517471da5262efbcc6b8`
- `xq_p4_mission`、`impact_mission` 和 `xq_p4_external_nav` 的 console entrypoint 及 Python 模块均从该安装树解析。

## 失败样本首因链

`8d992f42` 的首个数据偏离不是 Gazebo 段错误，也不是 ROS context 自行崩溃：

1. 起飞后真值高度持续上升，约达到 8 m，超过结构化房间约 4 m 的顶部；
2. `/livox/lidar` 仍在发布消息，但有限点数量逐步降为 0，随后点坐标全部为 `inf`；
3. FAST-LIO 连续输出 `No point, skip this scan!`；
4. `/localization/odom` 与 `/uav1/mavros/odometry/out` 同时中断约 20.30 s；
5. ExternalNav 随后报告 `source_stale`，飞控再出现 EKF variance / EKF LAND 等状态；
6. 清理过程正常结束，未发现本轮 Gazebo 段错误或 core dump。

IMU 和 `/clock` 在该窗口仍持续，故障链更符合“飞行高度超出 LiDAR 有效视场后定位停流”，而不是整个仿真进程崩溃。为什么该通用入口会产生这条爬升轨迹，仍需在使用专用 P4 入口的下一次单因素验证中继续核对，当前不凭此样本修改传感器或飞控参数。

## 本轮代码修复

- 运行时构建校验现在同时解析 `xq_p4_external_nav`、`xq_p4_mission` 和 `impact_mission` 的实际入口及模块路径；入口混用或安装树不匹配会在启动前拒绝。
- 专用 P4 飞行中健康丢失时，若收到的新鲜 FCU 状态表明已解锁且不在 LAND，进入 LAND 终止流程。
- 若 FCU 已经处于 LAND，不再请求 GUIDED、起飞或切换模式，保留故障并等待终止证据。
- FCU 状态过期或不可确认时，不把“未解锁”当成终止成功；结果保留 `termination unconfirmed`。
- 新增对应状态机测试，覆盖健康丢失、已有 LAND、状态过期和未知状态。

## 验证状态

- 专项 P4/运行时测试：`29 passed`
- 完整仓库回归：`192 passed`
- 修复后的安装树绑定检查：通过
- 不解锁冒烟：上一冻结版本已通过；本轮修复后尚未重跑
- 专用 P4 三次冷启动：本轮尚未重跑；历史 `p4_health_gate_dev_03`—`05` 继续保留，不与新版本混合

下一步只能从当前最终安装树启动一次不解锁观测，随后依据首个新证据决定是否建立新的专用 P4 验证组。不得用反复重试凑三次 PASS。
