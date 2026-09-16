# CPU P4 启动对照诊断（2026-09-16）

## 结论状态

本报告只对照两次历史 CPU P4，不改变原始运行目录，也不把一次成功和一次失败组成验证组。
两次实际加载的构建清单均为 `4ffa7a6adf8199b11347d4d7c2465600cb24b4a8`，源码哈希均为
`d0e2162b59c927bed1e6ec5e4696dfe6b8b9d45d514b5b47588a46ca15fcc5ee`，不是 `865990a`。

| 运行 | 原始结果 | 更正后的说明 |
|---|---|---|
| `p4_20260916T054248Z_157661` | PASS | `4ffa7a6`；第一次 ARM 因 `Gyros inconsistent` 被拒，2.10 s 后第二次 ARM 成功，随后完成任务与终止确认 |
| `p4_20260916T054523Z_159319` | FAIL | `4ffa7a6`；第一次 ARM 同时报告 `Gyros inconsistent` 与 `VisOdom: not healthy`，未解锁，终止确认完整 |

原始文件保持不变。机器可读对照结果在
`experiments/diagnostics/cpu_p4_20260916/comparison.json`，其中记录了原始关键文件哈希。

## Gyros inconsistent

ArduPilot 日志和参数表表明两次均启用了 IMU/gyro 实例 0 与 1，设备 ID 分别为
`2752772`、`2752780`。`INS_USE=1`、`INS_USE2=1`，不存在第三个实际 gyro 设备。

ArduPilot 的该门禁有两个条件：任一启用 gyro 与主 gyro 的三轴矢量差超过 5 deg/s 会重置
一致性计时；所有启用 gyro 还必须连续一致至少 10 s。告警文字不区分“瞬时超过阈值”和
“连续一致时间尚不足”。

两次均为 `LOG_DISARMED=0`，所以 DataFlash 未保存第一次 ARM 之前的 IMU 原始数据。已有记录中：

- PASS 运行记录到的 ARM 前 14 对 IMU 0/1 样本最大差为 0.0251 deg/s；
- FAIL 运行记录到的 11 对样本最大差为 0.0213 deg/s；
- 两者都远低于 5 deg/s，但这些样本从 ARM 尝试附近才开始，不能反证更早发生过瞬时差异，
  也不能证明 10 s 连续计时已经完成。

因此当前只能确定告警涉及 IMU 0/1，不能从旧证据确定究竟是阈值越界还是 10 s 计时未满。
PASS 样本在 2.10 s 后无需改参数即可解锁，更支持“启动期门禁尚未稳定或短暂被重置”这一假设，
但还不是证明。

## ExternalNav 与飞控 VisOdom

两套健康定义不同：

- 适配器健康检查 ROS 输入的新鲜度、输入频率、时间戳单调性和 MAVROS 订阅端是否存在；
- ArduPilot `VisOdom` 健康只要求飞控后端在最近 300 ms 内实际收到视觉里程计消息。

FAIL 运行在 ARM 告警前 10 s 内，ROS 侧 `/uav1/mavros/odometry/out` 有 72 条消息，最大墙钟间隔
0.155 s；适配器 51 个状态全部健康，告警前报告的源数据年龄为 0.066 s。与此同时，DataFlash
只记录了 2 条 `VISP`，其远端消息时间分别比 ARM 告警早 6.716 s、6.576 s，DataFlash 记录随后
延续约 0.397 s，已经超过 ArduPilot 的 300 ms 健康窗口。

这说明 ROS 发布端健康不能证明 FCU 接收端健康，旧失败更符合 MAVROS/链路的短时积压或断流。
但 `LOG_DISARMED=0` 使 DataFlash 窗口从 ARM 尝试附近才开始，尚不能区分：

1. MAVROS odometry 回调排队或发送停顿；
2. MAVLink 链路/FCU 接收处理停顿；
3. DataFlash 启动边界造成的不完整观测。

两次都出现较高 timesync RTT（PASS 812.62 ms，FAIL 880.94 ms）和 `Wrong FCU time`，因此不能把
timesync 告警单独作为 FAIL 的根因。

## 初始状态、参数与启动顺序

- 两次使用相同构建清单、运行依赖哈希、world、参数文件、固定 bridge 随机种子和 CPU llvmpipe 配置。
- SITL 都在各自独立的 `sitl_runtime` 目录中以 `--wipe` 启动；不存在跨运行复用同一 `eeprom.bin`。
  两个最终 EEPROM 哈希不同是运行后状态不同，不是输入持久化文件复用。
- DataFlash 参数仅有 `BARO1/2_GND_PRESS`、`MOT_THST_HOVER`、`STAT_FLTTIME`、`STAT_RUNTIME` 不同。
  前两项是本轮气压初始化，后三项受是否完成飞行影响；未发现 ExternalNav、IMU 使用、校准或健康阈值差异。
- 初始仿真真值位置一致。仿真/墙钟比为 PASS 0.7097、FAIL 0.7053，差异较小。
- EKF 两实例在 PASS 的仿真时刻约 5.16 s 开始使用 ExternalNav；FAIL 为约 9.77 s，晚约 4.61 s。
  两次 origin 设置分别约 11.92 s、12.22 s，首次 ARM 告警均约 21.8 s。

## 下一步诊断门禁

先完成构建绑定后的一次不解锁 CPU 冒烟。除原有持续输出、端点、bag 和清理检查外，应补充：

- 启动即记录 DataFlash（只增加日志可见性，不关闭任何解锁检查）；
- 记录飞控预解锁健康状态，至少覆盖两个 gyro 的原始/滤波数据、一致性门禁建立时间；
- 记录 MAVROS 实际发送的 ODOMETRY 序列/时间与 FCU `VISP/VISV` 接收时间，区分发布、发送、接收；
- 不发送 ARM/TAKEOFF，观察 `VisOdom` 是否持续健康、何时建立及是否自行丢失；
- 只在上述证据形成单一可检验假设后改变一个因素，再建立新的 P4 验证组。

当前不进入阶段 A，不修改 Frontier、完整性算法、恢复策略，也不以等待更久作为既定修复。
