# STM32F407 电机驱动移植：软件框架与实施步骤（v1.0）

> 适用前提：板子已画好；芯片 STM32F407VET6；双 CAN、4 键、ST7789 240×240、
> W25Q64、24C02、硬件急停已确定。
>
> 本文只讲**软件怎么搭、按什么顺序做、每步怎么验收**。
> 设计细节（协议、MIT/CSP、重力、归零、存储）见
> `docs/STM32F407_电机驱动移植方案.md`。

---

## 0. 已冻结的决策

| 项 | 决策 |
|---|---|
| 坐标空间 | 录制/回放/归零/门禁**全部用电机 raw**；不导入标定表 |
| 电机数 | 16 轴（14 臂 + 2 夹爪）；探测到 14 轴只做基础控制 |
| 归零 | 用户自己摆参考姿态，**16 台电机全部写类型 6**；写后 raw=0 |
| 录制/回放门禁 | 16 轴 \|raw\| ≤ 15° 且姿态稳定 500 ms，否则拒绝 |
| 重力补偿 | **保留**；`direction` 从 `calibration.yaml` 固化 16 个 ±1 |
| 调度 | **裸机 + 1 ms 协作式调度**，不用 RTOS |
| UI | 中文 + 4 键（UP/DOWN/OK/BACK），字库子集放 W25Q64 |
| 安全 | 硬件急停切断电机动力 + 软件失能；急停恢复后重新初始化 |
| 板子定位 | 完全独立，不接 ROS |

---

## 1. 软件框架

### 1.1 分层

```text
┌──────────────────────────────────────────────────────────────┐
│ UI          LVGL 页面 / 4 键导航 / 中文提示 / 蜂鸣器           │
├──────────────────────────────────────────────────────────────┤
│ 应用        app_state：MIT / CSP / 录制 / 回放 / 归零 / 设置   │
├──────────────────────────────────────────────────────────────┤
│ 控制        mit_ctrl / csp_ctrl / gravity / safety            │
│             100 Hz，纯计算，不碰 HAL                           │
├──────────────────────────────────────────────────────────────┤
│ 电机        motor_mgr / rs_protocol / can_bus                 │
│             帧编解码、发送队列、反馈快照、超时/故障             │
├──────────────────────────────────────────────────────────────┤
│ 运动        recorder / playback / traj_store                  │
├──────────────────────────────────────────────────────────────┤
│ 存储/外设   w25q64 / at24c02 / st7789 / keys / buzzer / power  │
├──────────────────────────────────────────────────────────────┤
│ 平台        STM32 HAL + 1 ms SysTick 协作调度 + LVGL            │
└──────────────────────────────────────────────────────────────┘
```

**依赖方向**：UI → 应用 → 控制 → 电机 → HAL。
控制层只调用 `motor_mgr` 的接口，不直接操作 CAN 寄存器；
这样协议/控制/重力都能在 PC 上做主机侧单测。

### 1.2 主循环与时间片

```c
for (;;) {
  if (tick_10ms) { tick_10ms = 0; control_slot(); }  // 最高优先
  can_rx_drain();       // 每圈最多 N 帧
  storage_slice();      // W25Q64 状态机，一次一步
  ui_slice();           // LVGL 定时器 + 局部刷新
  key_scan();           // 20 ms
  monitor_slot();       // 1 s
  __WFI();
}
```

| 时间片 | 周期 | 预算 | 内容 |
|---|---|---|---|
| `control_slot` | 10 ms | ≤ 3 ms | 安全判定 → 控制律 → 组帧 → CAN 发送 |
| `can_rx_drain` | 事件/每圈 | ≤ 0.5 ms | 解析 type-2/24，更新快照 |
| `storage_slice` | 每圈 | ≤ 1 ms | 录制落盘、回放预取、flash BUSY 轮询 |
| `ui_slice` | 20~33 ms | ≤ 3 ms | LVGL timers、脏区渲染、按键事件 |
| `key_scan` | 20 ms | < 0.2 ms | 消抖、长短按 |
| `monitor_slot` | 1 s | < 1 ms | 温度/故障/电压、黑匣子、喂狗 |

纪律：**主循环里不能忙等**。W25Q64 擦/写只发命令，下一圈查 BUSY；
LCD 用 DMA；CAN 发送走队列 + 每毫秒 2~3 帧节流。

### 1.3 状态机

```text
BOOT → SELF_TEST → IDLE（全部失能）
IDLE ──选择──► MIT_ZERO / MIT_DRAG / MIT_HOLD
             ├► CSP_HOLD / PLAYBACK
             ├► RECORD（零力矩/重力拖动）
             ├► ZERO（16 台类型 6）
             └► SETTINGS / INFO
任意状态 ──急停 / 故障 / 反馈超时──► FAULT（全失能）
```

- 模式互斥：`app_state` 每圈只调用当前模式的一个 handler，不需要锁。
- 任何模式切换：先失能 → 按目标模式序列进入 → 斜坡。
- 急停：硬件断电 + 软件锁存；恢复后重新探测、重写参数、重新使能。

---

## 2. 关键数据结构

```c
/* 静态配置，由 gen_robot_config.py 生成 */
typedef struct {
  uint8_t can_bus;      // 0=CAN1, 1=CAN2
  uint8_t can_id;
  uint8_t model;        // RS00..RS06
  int8_t  direction;    // ±1，仅重力补偿用
  float   max_torque;   // N·m
  float   max_speed;    // rad/s
  float   kp_hold, kd_hold;
  float   limit_spd, acc_rad, limit_cur;
  bool    is_gripper;
} motor_cfg_t;

/* 运行期反馈 */
typedef struct {
  float raw_pos, raw_vel, raw_torque;
  float temperature;
  uint8_t fault_bits;
  uint32_t last_rx_ms;
  bool online;
  bool enabled;
} motor_state_t;

/* 100 Hz 指令，raw 空间 */
typedef struct {
  float raw_pos, raw_vel;
  float kp, kd, tau_ff;
} mit_cmd_t;

/* 录制/回放帧 */
typedef struct {
  int16_t raw[16];      // 2500 counts/rad
} traj_frame_t;
```

> `direction` 只用于 `q = direction × raw` 的重力计算；
> 录制/回放/CSP 全部直接用 raw。

---

## 3. 模块接口（先定这些，后面各模块可并行）

```c
/* motor_mgr */
void     motor_init(void);
int      motor_probe(void);                  // 返回在线数
int      motor_enable_all(void);
int      motor_disable_all(void);
int      motor_write_mit(const mit_cmd_t cmd[16]);
int      motor_write_csp(const float raw[16]);
const motor_state_t *motor_state(void);      // 快照
bool     motor_all_online(void);

/* gravity */
void     gravity_init(void);
void     gravity_calc(const float raw[16], float tau[16]);

/* recorder */
bool     recorder_start(uint8_t slot);
void     recorder_stop(bool save);
bool     recorder_is_active(void);

/* playback */
bool     playback_start(uint8_t slot, uint16_t interval_ms, uint16_t loops);
void     playback_stop(void);

/* traj_store */
int      traj_list(traj_info_t out[5]);
int      traj_load_header(uint8_t slot, traj_header_t *h);

/* zero */
int      zero_check_ready(void);              // 16 台在线/失能
int      zero_all_motors(void);               // 类型 6 + 验证 + pending
```

---

## 4. 实施步骤（S0 → S11）

### 阶段划分

```text
S0 冻结配置 ─┬─ S1 协议 C + 单测 ─┬─ S3 CAN/电机管理 ─┬─ S4 MIT 模式 ─┬─ S6 CSP/切换 ─┐
             │                    │                   │               │              ├─ S8 回放
             └─ S5 重力模型 ──────┘                   └─ S7 录制 ─────┴──────────────┘
S2 BSP/存储 ───────────────────────────────────────────┘
S9 归零 ── S10 UI/设置/安全 ── S11 整机/长稳
```

### S0 冻结配置（0.5~1 天）

**做什么**

- 从 `joint_spec.yaml` 取 16 轴的 CAN ID、型号、`max_safe_torque`、
  `limit_spd`、`acc_rad`、`current_limit`。
- 从 `calibration.yaml` 取 16 个 `direction = ±1`（只给重力用）。
- 生成 `robot_config.c/.h`：`motor_cfg_t g_motors[16]`、轴名表、CAN 总线分组。
- 同时生成一份 `robot_config.md` 供人工核对。

**验收**

- 16 行表逐项对照 `joint_spec.yaml`，ID/型号/direction 无差异。
- `is_gripper` 标记 18/28 两轴。
- 左右臂 CAN 分组正确（11~18 → CAN1，21~28 → CAN2）。

### S1 协议 C 版 + 主机单测（2~3 天）

**做什么**

- 把 `protocol.hpp` 翻成 `rs_protocol.c/.h`：
  - 29 位 ID 拼/解、MIT 编解码、类型 2/24 解码；
  - 参数读写（17/18）、使能/失能（3/4）、类型 6、类型 24；
  - 型号量程表、kt 表、`csp_limit_current_from_torque`（含 ×√2）。
- 主机侧 gcc 单测：
  - 用 `test_protocol.cpp` 的期望字节做逐字节比对；
  - 覆盖边界值（±量程、0、超限钳位）；
  - 覆盖 CSP `limit_cur` 的 √2 折算。

**验收**

- 所有帧与 ROS 版本逐字节一致。
- 单测通过，无需硬件。

### S2 BSP 与存储（2~3 天）

**做什么**

- 按实际原理图核对 `board_config.h`：时钟、CAN、SPI、I2C、TIM、GPIO。
- ST7789：SPI + DMA 点亮，LVGL 移植，240×24 双缓冲。
- 4 键消抖、长短按；蜂鸣器。
- W25Q64：读 ID、扇区擦、页写、BUSY 轮询；24C02 读写。
- 自检页：显示 16 轴在线状态、存储容量、固件版本。

**验收**

- 屏幕无花屏，局部刷新正常；按键无抖动误触。
- W25Q64/24C02 写入后掉电重读一致。
- 自检页能显示 CAN 错误计数。

### S3 CAN 与电机管理（3~4 天）

**做什么**

- CAN1/CAN2 初始化、滤波器（CAN1 0~13，CAN2 14~27）、中断收帧、发送队列。
- 探测：`GetDeviceId` 扫 ID，记录 UID；判定 14/16 轴。
- 读 raw：零 MIT 帧诱发 type-2；维护 `motor_state_t` 快照。
- 使能/失能：类型 3/4；失败重试；参数写入（run_mode/limits）。
- 超时/故障：单轴 > 200 ms、整臂 > 500 ms 的降级。

**验收**

- 16 台全部在线，raw 值与 ROS `set_zero --check` 一致（±0.01 rad）。
- 单轴拔线后 200 ms 内报故障并松劲。
- 使能/失能无丢帧，CAN 错误计数为 0。

### S4 MIT 模式（3~4 天）

**做什么**

- 零力矩：全 0；HOLD：捕获 raw + 0.5 s 斜坡；零力矩拖动。
- 力矩钳位、速度钳位；位置读不到时拒绝 HOLD。
- 模式切换：先失能再进；HOLD 斜坡防冲击。

**验收**

- 100 Hz 连续运行 30 min，周期抖动 < 2 ms。
- 手推 HOLD 有阻尼、松手回位；零力矩拖动无回复力。
- 掉线/故障自动写 0。

### S5 重力模型生成 + 板端拖动（3~5 天）

**做什么**

- `gen_gravity_model.py`：
  - 解析 `lite_arm_model.urdf`，提取每臂 7 个转动关节的变换/轴；
  - 把 `inertia_params.yaml` 的 mass/first_moment 写回；
  - 末端固定/夹爪/被动/相机连杆按 q=0 合并；
  - 生成 `gravity_model_left/right.c/h`。
- 主机侧对照：
  - Python 用 pinocchio 算 `rnea(q,0,0)`；
  - C 版用 gcc 编译，随机 100 组 q 对比，误差 < 1e-3 N·m。
- 板端：
  - `q_i = direction_i × raw_i`；
  - `τ_ff = clamp(ĝ(q) + offsets_nm, ±max_safe_torque) × ratio`；
  - Kp=0、Kd=0.5、0.5 s 斜坡。

**验收**

- C 与 pinocchio 误差 < 1e-3 N·m（目标 < 1e-4）。
- 任意姿态单手可拖动，松手漂移 < 1°/3 s（受摩擦限制）。
- 力矩不超 `max_safe_torque`；急停/掉线立即松劲。

### S6 CSP 与 MIT↔CSP 切换（3~4 天）

**做什么**

- CSP 进入序列：失能 → run_mode=5 → limit_spd/acc/limit_cur → 使能
  → 首帧 loc_ref 写当前 raw。
- CSP HOLD；loc_ref 100 Hz；参数 RAM 重写。
- 切换：CSP→MIT 失能 → run_mode=0 → 等 100 ms → MIT 斜坡。

**验收**

- 切换过程无跳变、无堵转故障。
- CSP 下 loc_ref 跟踪误差 < 0.02 rad。
- 参数上电/切换后生效（读回核对）。

### S7 录制（3~4 天）

**做什么**

- 16 轴 raw 100 Hz 采样；RAM 环 → W25Q64 扇区双缓冲。
- 5 个槽位；头部 + CRC + 完成标志；掉电恢复。
- ±15° 门禁（16 轴 + 稳定 500 ms）；不通过拒绝并提示。
- 两种拖动方式：零力矩 / 重力补偿。

**验收**

- 5 条各 ≥ 60 s 录制成功；断电后完整。
- 门禁超差时拒绝，提示具体轴。
- 录制中控制周期抖动 < 2 ms。

### S8 回放（3~4 天）

**做什么**

- 选槽 → 门禁 → 间隔/次数 → 预对准 → CSP 回放。
- 帧间线性插值；raw 直接写 loc_ref。
- 任意键/急停中止：保持当前位置（CSP HOLD），不失能。
- 循环、进度、当前帧号。

**验收**

- 回放跟踪误差 < 0.02 rad；次数/间隔生效。
- 中止不坠臂；越界帧拒绝并指出轴/帧号。

### S9 归零（2~3 天）

**做什么**

- 16 台（含夹爪）类型 6：失能 → 读 raw_before → pending → 写零
  → 回读验证 → 清 pending。
- 部分失败/回读不确定：维护态，只允许重试。
- UI：显示 16 轴 raw 角度；长按 OK 3 s；结果页。

**验收**

- 16 台写零后 `|raw| < 2°`；断电重上电保持。
- 中途拔线/掉电后，pending 能判断状态，不误提交。
- 写零后录制/回放 raw 正常。

### S10 UI/设置/安全（4~5 天）

**做什么**

- 4 键菜单：MIT 模式、CSP/回放、录制、归零、设置、信息、安全。
- 中文字库子集（W25Q64 按需读）；状态色、进度条、蜂鸣器。
- 设置持久化（24C02 + W25Q64）；参数写入电机 RAM。
- 硬件急停：辅助触点检测、锁存、恢复后重新初始化。
- 黑匣子：故障前 N 秒 raw/CAN 记录。

**验收**

- 全流程不接串口可完成：上电→自检→拖动→录制→回放→归零。
- 急停按下立即断电、屏幕锁存；恢复后重新探测/使能正常。
- 设置掉电保持。

### S11 整机与长稳（3~5 天）

**做什么**

- 双 CAN 长稳；故障注入（拔线、过温模拟、总线错误）。
- 录制/回放 100 次循环；存储磨损评估。
- 文档：接线、操作、故障码、维护。

**验收**

- 连续 1 h 无 CAN 错误、无丢帧。
- 故障注入均能安全降级。
- 操作手册可让非开发人员完成录制/回放。

---

## 5. 并行与关键路径

| 可并行 | 说明 |
|---|---|
| S0 / S1 / S5 生成器 | 不依赖硬件，可立即开始 |
| S2 BSP | 依赖实际板子 |
| S3 | 依赖 S1 + S2 |
| S4 / S5 板端 | 依赖 S3 |
| S6 | 依赖 S3 + S4 |
| S7 | 依赖 S2 + S4/S5 |
| S8 | 依赖 S6 + S7 |
| S9 / S10 | 依赖 S3 + S7 |
| S11 | 最后 |

关键路径：**S0 → S1 → S3 → S4 → S5 → S7 → S8 → S11**。
S5（重力）和 S7（录制落盘）是最容易拖的两段，建议提前做主机侧验证。

---

## 6. 主机侧测试清单（不接板也能做）

| 测试 | 输入 | 判据 |
|---|---|---|
| 协议帧 | `test_protocol.cpp` 向量 | 逐字节一致 |
| MIT 量化 | ±量程/0/超限 | 钳位正确、无溢出 |
| CSP 电流折算 | 扭矩/kt | 含 ×√2，误差 < 1e-6 |
| 重力模型 | 100 组随机 q | C vs pinocchio < 1e-3 N·m |
| 轨迹编解码 | int16 raw | 量化误差 < 0.0005 rad |
| 门禁逻辑 | 15°/15.1° | 边界拒绝正确 |
| CRC/槽位 | 损坏数据 | 拒绝加载 |

---

## 7. 风险与应对（实施层）

| 风险 | 应对 |
|---|---|
| 板子引脚与文档不一致 | 第一步核对原理图，只改 `board_config.h` |
| CAN 收发器/终端问题 | 先单总线单电机，再双总线 16 台 |
| flash 擦写卡控制 | 非阻塞状态机 + 双扇区 + 只在 storage_slice 推进 |
| UI 全屏刷新卡控制 | 只在 IDLE 全屏；运行中只刷小区域 |
| 重力方向/零位不对 | 先台架对比 pinocchio；小比例 ratio 起步 |
| 录制数据损坏 | 头部 CRC + 完成标志 + 帧数上限 |
| 归零部分失败 | pending 存证 + 逐台验证 + 维护态 |
| 急停后电机重启 | 重新探测、重写 run_mode/limits、重新使能 |

---

## 8. 现在就能开始的三件事

1. **S0**：生成 `robot_config.c/h` + 核对表（ID/型号/direction/上限）。
2. **S1**：`rs_protocol.c/h` + 主机侧逐字节单测。
3. **S5a**：`gen_gravity_model.py` + C 版重力函数 + 与 pinocchio 的
   100 组随机位形对照。

这三件都不依赖板子，做完之后再上板做 S2/S3，节奏最稳。

---

*本文 v1.0，对应主方案 v1.0；板子已画好，后续以实际原理图核对引脚为准。*
