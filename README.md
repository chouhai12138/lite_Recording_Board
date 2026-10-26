# STM32F407 双 CAN 机械臂录制板（Lite Recording Board）

> 一块独立的、不依赖 ROS 的机械臂**示教录制**控制器：双路 1 Mbit/s CAN 驱动 16 台 RobStride 关节电机，
> 1.54" 屏 + 4 键中文界面，零力矩拖动示教，动作存 W25Q64，一键回放。
> 裸机实现，100 Hz MIT 控制 + 重力补偿。

<p>
  <img alt="MCU" src="https://img.shields.io/badge/MCU-STM32F407VET6-blue">
  <img alt="Clock" src="https://img.shields.io/badge/SYSCLK-168%20MHz-green">
  <img alt="CAN" src="https://img.shields.io/badge/CAN-2%C3%97%201%20Mbit%2Fs-orange">
  <img alt="Control" src="https://img.shields.io/badge/Control-100%20Hz%20MIT%20%2B%20CSP-purple">
  <img alt="UI" src="https://img.shields.io/badge/UI-1.54%22%20ST7789%20%2B%204%20Keys-lightgrey">
  <img alt="License" src="https://img.shields.io/badge/License-MIT-brightgreen">
</p>

---

## 目录

- [这是什么](#这是什么)
- [功能特性](#功能特性)
- [硬件与接线](#硬件与接线)
- [目录结构](#目录结构)
- [快速开始](#快速开始)
- [使用说明](#使用说明)
- [实现要点](#实现要点)
- [测试](#测试)
- [已知限制](#已知限制)
- [路线图](#路线图)
- [文档与致谢](#文档与致谢)

---

## 这是什么

给 7 轴 + 夹爪（双 7 轴）机械臂做的一块**离线示教器**：

1. 手掰机械臂 → 记录轨迹（**零力矩 + 阻尼**，不带重力补偿，最接近"直接拖"的手感）；
2. 轨迹实时写入板载 **W25Q64**，5 个槽位、每槽 **60 秒**（100 Hz 采样），掉电不丢；
3. 回放支持 **CSP**（`loc_ref` + 速度/加速度/电流限幅）或 **MIT**（kp/kd 位置跟踪）两种方式，可设轮数与轮间停留；
4. 另带**重力补偿拖动**模式（解析重力模型，和 pinocchio `rnea` 误差 < 1e-4 N·m）；
5. 全部操作可在 **1.54" 屏幕 + 4 个按键**上完成，也可通过串口命令行调试。

典型用途：现场快速示教一段动作、反复回放验证、拍视频，不需要连工控机。

---

## 功能特性

### 运动控制

| 功能 | 说明 |
|---|---|
| 双 CAN | CAN1 / CAN2 各 8 轴，1 Mbit/s，29 位扩展帧，FIFO0 中断接收 |
| 控制节拍 | **100 Hz**（实测 200 拍 / 2.000 s，抖动窗口 min/max/avg = 10/10/10.00 ms） |
| MIT 模式 | `ZERO`（零力矩 + 逐轴阻尼）/ `DRAG`（重力补偿 + 阻尼）/ `HOLD`（位置保持，Kp 0.5 s 斜坡） |
| 重力补偿 | 与 `gravity_compensator.cpp` 同源的解析实现，`max_err ≈ 7e-5 N·m`（200 组随机位形） |
| CSP 模式 | 回放用，逐轴限速/限加速/**限流按 `max_safe_torque` 换算**（rs-02 8.11 A / rs-05 2.26 A …） |
| 安全 | 力矩钳位、NaN 回退、200 ms 无反馈判掉线并失能、表外电机 ID 告警、无在线电机拒录 |

### 录制 / 回放

| 项 | 规格 |
|---|---|
| 槽位 | 5 个，每槽 256 KB |
| 时长 | **60 s @ 100 Hz**（6000 帧） |
| 量化 | int16，`0.0005 rad/LSB`（≈0.03°） |
| 存储 | 边录边写 **W25Q64**（SPI1），每槽带 magic + 帧数 + CRC32 头 |
| 回放 | CSP / MIT 可选；次数 `0 = 无限`；轮间停留 0~10000 ms；一阶低通平滑（默认 τ=40 ms） |
| 收尾 | 回放结束自动切 `ZERO`（零力矩 + 阻尼），电机保持使能、不掉臂 |

### 界面

- 1.54" ST7789 240×240（SPI2），**中文界面**（自建 262 字形点阵字库，ASCII 8×16 + 汉字 16×16）；
- 4 键：`上 / 下 / 左(返回) / 右(确认)`；
- **增量刷新**：只重画变化的行与提示条，录制计时时屏幕不再整屏闪烁；
- 串口文本后端可切换，方便远程调试。

---

## 硬件与接线

### 主控

| 项 | 值 |
|---|---|
| MCU | STM32F407VET6（Cortex-M4F，512 KB Flash / 128 KB RAM / 64 KB CCM） |
| 时钟 | HSE 25 MHz → PLL 168 MHz，APB1 42 MHz / APB2 84 MHz，Flash latency 5 |
| 调试/烧写 | WCH-Link（CMSIS-DAP v2，`1a86:8012`）或 DAPLink / ST-Link |

### 外设接线

| 功能 | 引脚 |
|---|---|
| CAN1 | **PB8 = RX / PB9 = TX**（AF9） |
| CAN2 | **PB12 = RX / PB6 = TX**（AF9） |
| LCD（SPI2） | SCK **PB13** / MOSI **PB15** / CS **PB14** / DC **PD12** / BL **PD11** |
| W25Q64（SPI1） | CS **PA15** / SCK **PA5** / MOSI **PB5** / MISO **PB4**（AF5） |
| 按键 | 上 **PE14** / 右 **PE12** / 下 **PE10** / 左 **PE8**（低有效，内部上拉） |
| 调试串口 | USART1 **PA9 = TX / PA10 = RX**，115200 8N1 |
| 指示灯 | CAN **PD15** / FAULT **PD13** |

> ⚠️ `PA15` 在核心板上同时接 LED 与 W25Q64 的 `/CS`，本工程归 W25Q64 使用（访问 Flash 时那颗 LED 会闪）。
> `PD11/PD12/PB13/PB14/PB15` 已分配给 LCD。

### 电机

| 轴 | CAN ID | 型号 | `max_safe_torque` |
|---|---|---|---|
| 左臂 1~7 | 11~17 | rs-02 / rs-00 ×3 / rs-05 ×3 | 7.0 / 7.0 / 2.5 / 4.0 / 1.5 / 1.8 / 1.8 N·m |
| 左夹爪 | 18 | rs-05 | 0.25 N·m |
| 右臂 1~7 | 21~27 | 同上 | 同上 |
| 右夹爪 | 28 | rs-05 | 0.25 N·m |

> 电机 11~18 挂在 **CAN2**、21~28 挂在 **CAN1**（实测线束如此）。总线两端需 120 Ω 终端电阻。

---

## 目录结构

```
.
├── Core/                  启动、时钟、中断、main（命令解析）
├── BSP/                   板级驱动
│   ├── Src/               led / keys / uart / lcd / can / w25q
│   └── lcd154/            原厂 1.54" ST7789 驱动（逐字保留，勿改）
├── App/
│   ├── motor/             rs_protocol（RobStride 协议）、can_bus、motor_mgr
│   ├── control/           mit_ctrl（100 Hz 控制律）、gravity（重力模型）
│   ├── motion/            traj（录制/回放 + W25Q64 存储）
│   ├── ui/                ui_core（页面树）、ui_lcd、ui_text、zh_font（中文点阵）
│   └── cfg/               生成的重力模型表、关节映射
├── Sim/                   虚拟 CAN 总线（16 台假电机，可离线开发）
├── Scripts/               重力模型生成、中文点阵生成
├── Tests/                 协议单测、重力模型对照测试
├── tools/                 flash.sh、udev 规则、OpenOCD 配置、SVD
└── DOCS/                  原理图、开发清单、重力排查笔记
```

---

## 快速开始

### 1. 依赖

```bash
sudo apt install gcc-arm-none-eabi make openocd gdb-multiarch picocom
```

### 2. 编译

```bash
make -j$(nproc)      # 产出 build/firmware.elf / .hex / .bin
make size            # 查看占用
make test            # 主机侧单元测试（协议 177 断言 + 重力 200 组对照）
```

默认 `HSE=25000000`、`UART_BAUD=115200`，可用 `make HSE=... UART_BAUD=...` 覆盖。

### 3. 烧写

```bash
./tools/install-udev.sh          # 首次：给 WCH-Link / DAPLink 放开权限
./tools/flash.sh auto build/firmware.elf   # 自动识别探针（dap/wch/stlink/jlink）
PROBE=wch ./tools/flash.sh auto build/firmware.elf   # 强制指定
```

### 4. 看串口

```bash
picocom -b 115200 /dev/ttyACM0    # 退出：Ctrl-A 再 Ctrl-X
```

### 5. 上电自检

```
 CAN   : 真实双 CAN 1Mbit/s，电机表固定 id=11-18(CAN2) / 21-28(CAN1)
 CAN   : 在线 16/16
 MODE  : DRAG 零力拖动（重力补偿 + Kd 阻尼）
 FLASH : W25Q64 OK (JEDEC 0xEF4017)，载入 N 个有效槽位
 LCD   : 1.54" ST7789 240x240 OK，UI 输出到屏幕
```

---

## 使用说明

### 屏幕上（4 键）

```
主菜单
├─ 录制   → 槽位 1-5 / 开始录制 ⇄ 停止录制 / 返回
├─ 回放   → 槽位 / 方式(0=CSP 1=MIT) / 次数(0=无限) / 间隔(ms) / 开始回放 / 停止 / 返回
├─ 模式   → 零力矩(阻尼) / 重力补偿拖动 / 位置保持 / 返回
├─ 归零   → 全部归零（类型 6 + 回读校验）/ 返回
└─ 信息   → 模式、在线/使能数、槽位帧数、运行时间、按键状态
```

- `上/下` 选择，`右` 进入或开始，`左` 返回；
- 编辑数值：`上/下` 调整，`右` 确认，`左` 取消；
- 录制中底部提示实时显示 `录制中 x.xs (N 帧) 右:停止`。

### 串口命令（节选）

| 命令 | 说明 |
|---|---|
| `help` | 命令总表 |
| `motor status` | 16 轴状态（先自动读一轮：id/总线/在线/角度/速度/力矩/温度/故障位） |
| `motor zero` | 全部归零（失能 → 类型 6 → 回读校验） |
| `motor discover 1 64` / `motor adopt` | 扫描电机 ID 并按结果重建表 |
| `ctrl zero\|hold\|drag` | 切换控制模式 |
| `ctrl kp <v>` / `ctrl kd <v>` / `ctrl dragkd <v>` | 增益与拖动阻尼 |
| `ctrl grav` | 打印当前姿态与重力前馈力矩 |
| `ctrl kdtable` | 打印逐轴阻尼表 |
| `ctrl jitter` | 控制周期统计（min/max/avg/超时次数） |
| `rec <槽位>` / `rec stop` | 录制（自动进 ZERO 模式） |
| `play <槽位> [次数(0=无限)] [间隔ms]` | 回放 |
| `traj` / `traj dump <槽位>` / `traj mode csp\|mit` / `traj smooth <ms>` | 轨迹查看、回放方式、平滑 |
| `can` / `can test` / `can sniff on\|off` | 双 CAN 状态、回环自测、原始帧打印 |
| `key up\|down\|left\|right` | 模拟按键（远程调试 UI） |
| `ui text` / `ui lcd` | 切换 UI 后端 |

### 一段完整的示教

```bash
motor status                 # 确认 16/16 在线
rec 1                        # 进入零力矩模式 → 擦除槽 1（约 2 秒）→ 开始录制
# 手掰机械臂 3~10 秒
rec stop                     # 写入 W25Q64
play 1 3 1000                # 回放 3 次，每次之间停 1 秒
```

---

## 实现要点

```
        ┌──────────────────────────── main (1 ms 协作式循环) ───────────────────────────┐
        │  按键扫描 → UI 事件 → CAN_Bus_Poll → MotorMgr_Tick → MitCtrl_Tick(100Hz)     │
        │                                    → Traj_RecordTick / Traj_PlayTick          │
        └───────────────┬───────────────────────────┬──────────────────────┬────────────┘
                        │                           │                      │
                  App/ui (页面树)            App/control (控制律)     App/motion (录制回放)
                        │                           │                      │
                  ui_lcd / zh_font           mit_ctrl + gravity         traj + W25Q64
                        │                           │                      │
                  BSP (lcd/keys/uart)         motor_mgr / can_bus      bsp_w25q (SPI1)
                                                    │
                                              bsp_can (SPI? no — bxCAN)
```

- **协议**：RobStride 29 位扩展帧，`ID = [28:24] 类型 | [23:8] 主机号 | [7:0] 电机 ID`；回包把**电机 ID 放在 `extra_data` 低字节**（实测，和发送方向相反）。
- **重力补偿**：`Scripts/gen_gravity_model.py` 从 URDF + 辨识参数生成关节链表，板端做前向运动学 + 解析重力矩，方向系数来自 URDF `<motorDirection>`。
- **录制格式**：每槽 `0x0000` 头部（magic/frames/CRC32/duration），`0x1000` 起数据；录制时按 256 B 页缓冲写入，避免阻塞采样。
- **UI 增量刷新**：屏幕缓存比对，只有变化的行/提示条重画。

---

## 测试

```bash
make test
# test_protocol: 177 checks, 0 failures  ALL PASS
# test_gravity : 200 samples, max_err = 6.9e-05 N·m (tol 1e-3), failures = 0
```

上板验证过的项：双 CAN 回环自测、16 轴发现/建表、角度回读、使能/失能、归类、录制 6000 帧、CSP/MIT 回放、重力补偿拖动、中文 UI + 4 键操作。

---

## 已知限制

1. **PA15 与 LED 复用**：W25Q64 的 `/CS` 与板载 LED 共线，Flash 访问时 LED 闪烁，属硬件设计；
2. **WCH-Link 串口桥**在持续输出时会丢字节（USB CDC 限制），调试时可能出现乱码，换 USB-TTL 可解决；
3. 归零后**旧轨迹失效**（零点变了），需要重新录制；
4. 目前只有 5 槽 × 60 s，尚未做「轨迹拼接 / 变速回放」；
5. 急停走软件（失能），硬件急停回路待接。

---

## 路线图

- [x] 板级 bring-up、双 CAN、16 轴管理
- [x] MIT 控制（ZERO / DRAG / HOLD）+ 100 Hz 周期
- [x] 重力补偿（解析模型，pinocchio 对照通过）
- [x] 录制 / 回放（60 s × 5 槽，W25Q64 掉电保存）
- [x] 中文 UI + 4 键操作
- [ ] 回放变速（0.25×~2×）与轨迹拼接
- [ ] 24C02 参数持久化（增益、槽位名、标定）
- [ ] 硬件急停与蜂鸣器提示
- [ ] 可选 LVGL 界面

---

## 文档与致谢

- `DOCS/开发步骤清单_v1.1.md`：A0~A9 / B0~B6 开发清单与验收标准
- `DOCS/重力补偿排查笔记.md`：重力模型的失败路径与最终方案
- `DOCS/SCH_STM32F4x_DAP_CORE_CAN485_原理图_*.pdf`：核心板原理图
- `配套测试程序/`：LCD 原厂例程（本工程 LCD 驱动的来源）
- 协议与标定参数参考 `lite_driver` / `lite_controller` / `lite_dynamics_calib`

## License

MIT（`Drivers/` 下的 ST HAL 与 `BSP/lcd154/` 下的原厂驱动遵循其原始许可）。
