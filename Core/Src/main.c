/**
 * @file    main.c
 * @brief   STM32F407VET6 录音板 —— 板级自检 + 串口交互
 *
 * 已验证外设：
 *   · HSE 25MHz → PLL 168MHz
 *   · USART1 PA9/PA10 115200：中断接收 + 行编辑 + 命令解析
 *   · LED：RUN=PA15 / CAN=PD15 / REC=PD12 / FAULT=PD13（低电平点亮）
 *   · 按键：UP=PD11 / DOWN=PB15 / LEFT=PB13(返回) / RIGHT=PB14(确认)
 *
 * 串口命令见 help。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_keys.h"
#include "bsp_lcd.h"
#include "can_bus.h"
#include "motor_mgr.h"
#include "sim_can.h"
#include "bsp_can.h"
#include "mit_ctrl.h"
#include "traj.h"
#include "bsp_w25q.h"
#include "ui.h"
#include "ui_backend.h"

#define LINE_MAX        64U
#define KEY_SCAN_MS     5U

static void SystemClock_Config(void);
static void UartPrintClock(void);
static void Prompt(void);
static void HandleLine(char *line);
static void ProcessKeyEvents(void);
static void LedCommand(const char *arg);
static void KeyCommand(const char *arg);
static void UiScreenCommand(const char *arg);
static void LcdCommand(const char *arg);
static void MotorCommand(const char *arg);
static void SimCommand(const char *arg);
static void CanCommand(const char *arg);
static void CtrlCommand(const char *arg);
static void LedsAllOff(void);

static char    s_line[LINE_MAX];
static uint8_t s_len;
static uint8_t s_got_cr;
static uint8_t s_esc;          /* 吞掉 ANSI 方向键序列 ESC [ x */

int main(void)
{
    uint32_t now, next_led_run, next_led_can, next_key_scan;
    uint32_t alive = 0;
    uint8_t  can_blink = 0;

    HAL_Init();
    SystemClock_Config();

    BSP_LED_Init();
    BSP_Keys_Init();
    BSP_UART_Init(UART_BAUD);
    LedsAllOff();

    MotorMgr_Init();        /* 电机管理 */
    MitCtrl_Init();         /* MIT 控制层（100Hz） */
    Traj_Init();            /* 录制/回放 */

    /* 先起 CAN 并等电机上电完成，再做开机扫描（电机启动比 MCU 慢） */
    if (CAN_Bus_SetHardware(1)) {
        printf(" CAN   : 真实双 CAN 1Mbit/s，电机表固定 id=11-18(CAN2) / 21-28(CAN1)\r\n");
        HAL_Delay(400);
        for (uint8_t attempt = 0; attempt < 3U; attempt++) {
            MotorMgr_Scan();
            if (MotorMgr_OnlineCount() > 0U) {
                break;
            }
            HAL_Delay(200);
        }
        printf(" CAN   : 在线 %u/%u\r\n",
               (unsigned)MotorMgr_OnlineCount(), (unsigned)MotorMgr_Count());
        if (MotorMgr_OnlineCount() > 0U) {
            /* 默认进入"带阻尼的零力拖动"（DRAG：重力补偿 + 逐轴 Kd） */
            MitCtrl_SetMode(MIT_MODE_DRAG);
            printf(" MODE  : DRAG 零力拖动（重力补偿 + Kd 阻尼）\r\n");
        }
        if (MotorMgr_UnknownCount() > 0U) {
            printf(" CAN   : ⚠ 总线上有表外电机（%lu 帧），检查 ID 是否 11-18/21-28\r\n",
                   (unsigned long)MotorMgr_UnknownCount());
        }
    } else {
        printf(" CAN   : 初始化失败，回退虚拟总线\r\n");
    }

    if (W25Q_Init()) {
        printf(" FLASH : W25Q64 OK (JEDEC 0x%06lX)，载入轨迹...\r\n",
               (unsigned long)W25Q_JedecId());
        printf(" FLASH : 载入 %d 个有效槽位\r\n", Traj_LoadAll());
    } else {
        printf(" FLASH : W25Q64 未识别 (JEDEC 0x%06lX)\r\n",
               (unsigned long)W25Q_JedecId());
    }

    printf("\r\n\r\n");
    printf("==========================================\r\n");
    printf(" STM32F407VET6 录音板 bring-up\r\n");
    printf(" build : %s %s\r\n", __DATE__, __TIME__);
    printf(" LED   : CAN=PD15 FAULT=PD13（PA15=W25Q64 CS, PD12=LCD DC）\r\n");
    printf(" KEY   : UP=PE14 DOWN=PE10 LEFT=PE8 RIGHT=PE12（底板，低有效）\r\n");
    UartPrintClock();
    printf(" 串口 UI：%lu bps，用 key 命令或板载按键操作（help）\r\n",
           (unsigned long)UART_BAUD);
    printf("==========================================\r\n");

    /* 屏幕：开机初始化（原厂驱动），然后用屏幕作为主 UI 后端 */
    BSP_LCD_Init();
    if (BSP_LCD_IsReady()) {
        UI_LCD_Register();
        printf(" LCD   : 1.54\" ST7789 %ux%u OK，UI 输出到屏幕\r\n",
               (unsigned)BSP_LCD_Width(), (unsigned)BSP_LCD_Height());
    } else {
        UI_Text_Register();
        printf(" LCD   : 初始化失败，UI 先走串口\r\n");
    }
    UI_Init();

    now          = HAL_GetTick();
    next_led_run = now + 200U;
    next_led_can = now + 1000U;
    next_key_scan = now;

    while (1) {
        uint8_t ch;

        /* ---- 按键扫描（5ms） ---- */
        now = HAL_GetTick();
        if ((int32_t)(now - next_key_scan) >= 0) {
            BSP_Keys_Scan(now);
            next_key_scan += KEY_SCAN_MS;
            if ((int32_t)(now - next_key_scan) > 50) {   /* 落后太多则重新对齐 */
                next_key_scan = now + KEY_SCAN_MS;
            }
        }

        /* ---- 串口输入：回显 + 行编辑 ---- */
        while (BSP_UART_ReadByte(&ch)) {
            /* 方向键会发 ESC [ A/B/C/D，直接吞掉，别当命令 */
            if (s_esc == 1U) {
                s_esc = (ch == '[') ? 2U : 0U;
                continue;
            }
            if (s_esc == 2U) {
                if (ch >= '@' && ch <= '~') {
                    s_esc = 0U;
                }
                continue;
            }
            if (ch == 0x1BU) {
                s_esc = 1U;
                continue;
            }

            if (ch == '\r' || ch == '\n') {
                if (ch == '\n' && s_got_cr) {
                    s_got_cr = 0;
                    continue;
                }
                s_got_cr = (ch == '\r') ? 1U : 0U;

                if (s_len == 0U) {
                    Prompt();
                    continue;
                }
                s_line[s_len] = '\0';
                printf("\r\n");
                HandleLine(s_line);
                s_len = 0U;
                Prompt();
            } else if (ch == 0x08U || ch == 0x7FU) {
                if (s_len > 0U) {
                    s_len--;
                    printf("\b \b");
                }
            } else if (ch >= 0x20U && ch < 0x7FU) {
                if (s_len < LINE_MAX - 1U) {
                    s_line[s_len++] = (char)ch;
                    BSP_UART_PutChar(ch);
                }
            }
        }

        /* ---- 按键事件 → UI ---- */
        ProcessKeyEvents();

        /* ---- 总线 & 电机控制（10ms HOLD 节拍 + 超时检测） ---- */
        CAN_Bus_Poll();
        MotorMgr_Tick(now);     /* 只做超时/掉线判定 */
        MitCtrl_Tick(now);      /* 100Hz MIT 控制 */
        Traj_RecordTick(now);   /* 录制采样 */
        Traj_PlayTick(now);     /* CSP 回放 */

        /* ---- UI 定时（消息超时、INFO 页刷新） ---- */
        UI_Tick(now);

        /* ---- LED ---- */
        if ((int32_t)(now - next_led_run) >= 0) {
            BSP_LED_Toggle(BSP_LED_CAN);
            next_led_run += 200U;
        }
        if ((int32_t)(now - next_led_can) >= 0) {
            can_blink = (uint8_t)!can_blink;
            BSP_LED_Write(BSP_LED_CAN, can_blink);   /* 暂无 CAN：慢闪表示活 */
            next_led_can += 1000U;
            alive++;
        }
    }
}

static void Prompt(void)
{
    printf("> ");
}

static void LedsAllOff(void)
{
    for (uint8_t i = 0; i < (uint8_t)BSP_LED_COUNT; i++) {
        BSP_LED_Off((bsp_led_t)i);
    }
}

static void ProcessKeyEvents(void)
{
    bsp_key_event_t ev;

    while (BSP_Keys_GetEvent(&ev)) {
        if (ev.ev == BSP_KEY_EV_RELEASE) {
            continue;                       /* 目前只用按下/长按 */
        }

        /* 底板还没到，只有 2 个键（UP=PA0 / DOWN=PD14），
         * 先用「长按 UP = 确认、长按 DOWN = 返回」把 UI 跑起来；
         * 底板左右键接好后把这里删掉即可。 */
        if (!BSP_Key_IsEnabled(BSP_KEY_LEFT) && !BSP_Key_IsEnabled(BSP_KEY_RIGHT) &&
            ev.ev == BSP_KEY_EV_LONG) {
            if (ev.key == BSP_KEY_UP) {
                UI_HandleKey(BSP_KEY_RIGHT, 0);
            } else if (ev.key == BSP_KEY_DOWN) {
                UI_HandleKey(BSP_KEY_LEFT, 0);
            }
            continue;
        }

        UI_HandleKey(ev.key, ev.ev == BSP_KEY_EV_LONG);
    }
}

static void LedCommand(const char *arg)
{
    char name[8];
    const char *sp;
    size_t n;
    bsp_led_t led;
    int found = 0;

    sp = strchr(arg, ' ');
    n = sp ? (size_t)(sp - arg) : strlen(arg);
    if (n >= sizeof(name)) {
        n = sizeof(name) - 1U;
    }
    memcpy(name, arg, n);
    name[n] = '\0';

    if (strcmp(name, "can") == 0)   { led = BSP_LED_CAN;   found = 1; }
    else if (strcmp(name, "fault") == 0) { led = BSP_LED_FAULT; found = 1; }

    if (!found) {
        printf("用法: led can|fault on|off|toggle\r\n");
        return;
    }
    if (!sp) {
        printf("用法: led can|fault on|off|toggle\r\n");
        return;
    }

    sp++;
    if (strcmp(sp, "on") == 0) {
        BSP_LED_On(led);
        printf("LED %s = ON\r\n", name);
    } else if (strcmp(sp, "off") == 0) {
        BSP_LED_Off(led);
        printf("LED %s = OFF\r\n", name);
    } else if (strcmp(sp, "toggle") == 0) {
        BSP_LED_Toggle(led);
        printf("LED %s = TOGGLE\r\n", name);
    } else {
        printf("用法: led can|fault on|off|toggle\r\n");
    }
}

static void KeyCommand(const char *arg)
{
    char name[8];
    const char *sp;
    size_t n;
    bsp_key_t key = BSP_KEY_UP;
    int found = 0;
    int is_long = 0;

    sp = strchr(arg, ' ');
    n = sp ? (size_t)(sp - arg) : strlen(arg);
    if (n >= sizeof(name)) {
        n = sizeof(name) - 1U;
    }
    memcpy(name, arg, n);
    name[n] = '\0';

    if (strcmp(name, "up") == 0)         { key = BSP_KEY_UP;    found = 1; }
    else if (strcmp(name, "down") == 0)  { key = BSP_KEY_DOWN;  found = 1; }
    else if (strcmp(name, "left") == 0)  { key = BSP_KEY_LEFT;  found = 1; }
    else if (strcmp(name, "right") == 0) { key = BSP_KEY_RIGHT; found = 1; }

    if (!found) {
        printf("用法: key up|down|left|right [long]\r\n");
        return;
    }
    if (sp && strcmp(sp + 1, "long") == 0) {
        is_long = 1;
    }
    BSP_Keys_Inject(key, is_long);
    printf("已注入按键 %s%s\r\n", name, is_long ? " (long)" : "");
}

/* newlib-nano 不支持 %f：这里把浮点转成"整数.一位小数"打印 */
static void print_fixed1(double v, uint8_t width)
{
    int    scaled = (int)(v * 10.0 + (v >= 0 ? 0.5 : -0.5));
    int    whole  = scaled / 10;
    int    frac   = scaled % 10;

    if (frac < 0) {
        frac = -frac;
    }
    if (width > 0U) {
        printf("%*d.%d", (int)width, whole, frac);
    } else {
        printf("%d.%d", whole, frac);
    }
}

static void MotorStatusTable(void)
{
    MotorMgr_Scan();            /* 先读一轮，保证打印的是最新值 */
    printf(" idx id bus on  pos(deg)  vel     trq     T    flt\r\n");
    for (uint8_t i = 0; i < MotorMgr_Count(); i++) {
        const motor_t *m = MotorMgr_Get(i);
        if (m == NULL) {
            continue;
        }
        printf(" %3u %2u  C%u %u  ", (unsigned)(i + 1U), (unsigned)m->id,
               (unsigned)(m->bus + 1U), (unsigned)(m->online ? 1U : 0U));
        print_fixed1(m->pos * 57.29578, 8);
        printf("  ");
        print_fixed1(m->vel, 6);
        printf("  ");
        print_fixed1(m->torque, 6);
        printf("  ");
        print_fixed1(m->temp, 4);
        printf("  0x%02X\r\n", (unsigned)m->fault_bits);
    }
    printf(" online=%u enabled=%u unknown_frames=%lu\r\n",
           (unsigned)MotorMgr_OnlineCount(), (unsigned)MotorMgr_EnabledCount(),
           (unsigned long)MotorMgr_UnknownCount());
}

static void MotorCommand(const char *arg)
{
    char     sub[16];
    char     arg2[16];
    const char *sp;
    size_t   n;

    sub[0] = '\0';
    arg2[0] = '\0';
    sp = strchr(arg, ' ');
    n = sp ? (size_t)(sp - arg) : strlen(arg);
    if (n >= sizeof(sub)) {
        n = sizeof(sub) - 1U;
    }
    memcpy(sub, arg, n);
    sub[n] = '\0';
    if (sp != NULL) {
        snprintf(arg2, sizeof(arg2), "%s", sp + 1);
    }

    if (strcmp(sub, "scan") == 0) {
        MotorMgr_Scan();
        printf("已向 %u 台电机发起读位置（backend=%s）\r\n",
               (unsigned)MotorMgr_Count(),
               CAN_Bus_IsHardware() ? "真实 CAN" : "虚拟总线");
        printf("  在线 %u / %u，状态用 motor status 看\r\n",
               (unsigned)MotorMgr_OnlineCount(), (unsigned)MotorMgr_Count());
        if (!CAN_Bus_IsHardware()) {
            printf("⚠ 当前是虚拟总线，读到的是假数据；接真电机请先执行 can hw\r\n");
        }
    } else if (strcmp(sub, "poll") == 0) {
        printf("向 %u 台发零力矩 MIT 帧（类型 1），用类型 2 回包刷状态\r\n",
               (unsigned)MotorMgr_Count());
        printf("⚠ 已使能的电机会松劲，请先 motor off all\r\n");
        MotorMgr_PollZeroMit();
        printf("  在线 %u / %u\r\n",
               (unsigned)MotorMgr_OnlineCount(), (unsigned)MotorMgr_Count());
    } else if (strcmp(sub, "discover") == 0) {
        int from = 1;
        int to = 32;
        const char *sp3;

        if (arg2[0] != '\0') {
            from = atoi(arg2);
            sp3 = strchr(arg2, ' ');
            if (sp3 != NULL) {
                to = atoi(sp3 + 1);
            } else {
                to = from + 15;
            }
        }
        printf("探测 CAN ID %d..%d（backend=%s）...\r\n", from, to,
               CAN_Bus_IsHardware() ? "真实 CAN" : "虚拟总线");
        MotorMgr_Discover((uint8_t)from, (uint8_t)to);
        printf("发现 %u 台：\r\n", (unsigned)MotorMgr_DiscCount());
        for (uint8_t i = 0; i < MotorMgr_DiscCount(); i++) {
            const motor_disc_t *d = MotorMgr_DiscAt(i);
            if (d == NULL) {
                continue;
            }
            printf("  id=%3u  bus=CAN%u  uid=%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                   (unsigned)d->id, (unsigned)(d->bus + 1U),
                   d->uid[0], d->uid[1], d->uid[2], d->uid[3],
                   d->uid[4], d->uid[5], d->uid[6], d->uid[7]);
        }
    } else if (strcmp(sub, "status") == 0) {
        int idx = atoi(arg2);
        if (idx >= 1 && idx <= (int)MotorMgr_Count()) {
            const motor_t *m = MotorMgr_Get((uint8_t)(idx - 1));
            if (m != NULL) {
                printf(" #%d id=%u online=%u enabled=%u pos=", idx,
                       (unsigned)m->id, (unsigned)m->online,
                       (unsigned)m->enabled);
                print_fixed1(m->pos * 57.29578, 0);
                printf("deg trq=");
                print_fixed1(m->torque, 0);
                printf(" T=");
                print_fixed1(m->temp, 0);
                printf(" flt=0x%02X\r\n", (unsigned)m->fault_bits);
            }
        } else {
            MotorStatusTable();
        }
    } else if (strcmp(sub, "move") == 0) {
        int idx = atoi(arg2);
        const char *sp2 = strchr(arg2, ' ');
        double deg = (sp2 != NULL) ? atof(sp2 + 1) : 0.0;

        if (idx >= 1 && idx <= (int)MotorMgr_Count()) {
            MotorMgr_MoveTo((uint8_t)(idx - 1), deg / 57.29578);
            printf("电机 %d 目标 %d°\r\n", idx, (int)deg);
        } else {
            printf("用法: motor move <1..16> <角度>\r\n");
        }
    } else if (strcmp(sub, "runmode") == 0) {
        printf("读所有电机的 run_mode (0x7005)...\r\n");
        MotorMgr_RequestParamAll(RS_PARAM_RUN_MODE);
        HAL_Delay(150);
    } else if (strcmp(sub, "locref") == 0) {
        printf("读所有电机的 loc_ref (0x7016)...\r\n");
        MotorMgr_RequestParamAll(RS_PARAM_LOC_REF);
        HAL_Delay(150);
    } else if (strcmp(sub, "zero") == 0) {
        int bad = MotorMgr_ZeroAllVerify();

        printf("提示：归零后旧轨迹的零点已变，建议重新录制\r\n");
        if (bad > 0) {
            printf("⚠ 有 %d 台没归到零：检查机械是否被挡住/编码器异常\r\n", bad);
        }
    } else if (strcmp(sub, "adopt") == 0) {
        if (MotorMgr_DiscCount() == 0U) {
            printf("没有缓存探测结果，先自动探测 1..64 ...\r\n");
            MotorMgr_Discover(1, 64);
        }
        uint8_t n = MotorMgr_Adopt();
        if (n == 0U) {
            printf("探测不到电机：确认总线/收发器/1Mbit/s/供电\r\n");
        } else {
            printf("已按探测结果重建电机表：%u 台\r\n", (unsigned)n);
            for (uint8_t i = 0; i < n; i++) {
                const motor_t *m = MotorMgr_Get(i);
                if (m != NULL) {
                    printf("  #%u id=%u bus=CAN%u\r\n", (unsigned)(i + 1U),
                           (unsigned)m->id, (unsigned)(m->bus + 1U));
                }
            }
            MotorMgr_Scan();
            printf("已向这 %u 台发读位置，稍后 motor status 看角度\r\n", (unsigned)n);
        }
    } else if (strcmp(sub, "on") == 0 || strcmp(sub, "off") == 0) {
        uint8_t on = (sub[1] == 'n') ? 1U : 0U;

        if (strcmp(arg2, "all") == 0 || arg2[0] == '\0') {
            MotorMgr_EnableAll(on);
            printf("已%s全部 16 台\r\n", on ? "使能" : "失能");
        } else {
            int idx = atoi(arg2);
            if (idx < 1 || idx > (int)MotorMgr_Count()) {
                printf("编号 1..%u\r\n", (unsigned)MotorMgr_Count());
                return;
            }
            MotorMgr_Enable((uint8_t)(idx - 1), on);
            printf("已%s 第 %d 台\r\n", on ? "使能" : "失能", idx);
        }
    } else if (strcmp(sub, "hold") == 0) {
        if (strcmp(arg2, "all") == 0 || arg2[0] == '\0') {
            for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                MotorMgr_Hold(i);
            }
            printf("全部电机 HOLD 到当前位置\r\n");
        } else {
            int idx = atoi(arg2);
            if (idx >= 1 && idx <= (int)MotorMgr_Count()) {
                MotorMgr_Hold((uint8_t)(idx - 1));
                printf("第 %d 台 HOLD\r\n", idx);
            }
        }
    } else {
        printf("用法: motor scan|on|off|zero|hold|status|move [all|编号] [角度]\r\n");
    }
}

static void CtrlCommand(const char *arg)
{
    uint32_t jmin, jmax, javg, jover;

    if (strcmp(arg, "zero") == 0) {
        MitCtrl_SetMode(MIT_MODE_ZERO);
        printf("MIT 模式 = ZERO（零力矩 + Kd=%.2f 阻尼）\r\n", MitCtrl_DragKd());
    } else if (strcmp(arg, "hold") == 0) {
        MitCtrl_SetMode(MIT_MODE_HOLD);
        printf("MIT 模式 = HOLD（kp=%.0f kd=%.1f）\r\n",
               MitCtrl_Kp(), MitCtrl_Kd());
    } else if (strcmp(arg, "drag") == 0) {
        MitCtrl_SetMode(MIT_MODE_DRAG);
        printf("MIT 模式 = DRAG（kp=0, kd[1]=%.2f；ctrl dragkd 查看逐轴）\r\n",
               MitCtrl_DragKdAt(0));
    } else if (strncmp(arg, "kp ", 3) == 0) {
        MitCtrl_SetGains(atof(arg + 3), MitCtrl_Kd());
        printf("Kp = %.1f\r\n", MitCtrl_Kp());
    } else if (strcmp(arg, "dragkd") == 0) {
        printf("DRAG 逐轴阻尼：\r\n");
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            printf(" %2u:%+.2f", (unsigned)(i + 1U), MitCtrl_DragKdAt(i));
            if ((i % 4U) == 3U) {
                printf("\r\n");
            }
        }
        printf("用法: ctrl dragkd <v> 或 ctrl dragkd <1..%u> <v>\r\n",
               (unsigned)MOTOR_COUNT);
    } else if (strncmp(arg, "dragkd ", 7) == 0) {
        const char *p = arg + 7;
        char *end = NULL;
        const double first = strtod(p, &end);
        while (*end == ' ') {
            end++;
        }
        if (*end != '\0') {
            /* ctrl dragkd <idx> <v>：idx 从 1 开始，和 motor 命令一致 */
            const int idx = (int)first;
            const double kd = strtod(end, NULL);
            if (idx >= 1 && idx <= (int)MOTOR_COUNT) {
                MitCtrl_SetDragKdAt((uint8_t)(idx - 1), kd);
                printf("DRAG 轴 %d Kd = %.3f\r\n", idx,
                       MitCtrl_DragKdAt((uint8_t)(idx - 1)));
            } else {
                printf("轴号 %d 越界（1..%u）\r\n", idx, (unsigned)MOTOR_COUNT);
            }
        } else {
            MitCtrl_SetDragKd(first);
            printf("DRAG 全部轴 Kd = %.3f\r\n", MitCtrl_DragKd());
        }
    } else if (strncmp(arg, "kd ", 3) == 0) {
        MitCtrl_SetGains(MitCtrl_Kp(), atof(arg + 3));
        printf("Kd = %.2f\r\n", MitCtrl_Kd());
    } else if (strcmp(arg, "kdtable") == 0) {
        printf("逐轴拖动阻尼 Kd（ZERO/DRAG 共用）：\r\n");
        for (uint8_t i = 0; i < 16; i++) {
            printf("  #%2u %.2f%s", (unsigned)(i + 1U), MitCtrl_DragKdAt(i),
                   ((i % 4U) == 3U) ? "\r\n" : "   ");
        }
    } else if (strcmp(arg, "grav") == 0) {
        MitCtrl_DebugGravity();
    } else if (strcmp(arg, "jitter") == 0) {
        MitCtrl_GetJitter(&jmin, &jmax, &javg, &jover);
        printf("控制周期: min=%lu max=%lu avg=%lu.%lums over=%lu ticks=%lu\r\n",
               (unsigned long)jmin, (unsigned long)jmax,
               (unsigned long)(javg / 10U), (unsigned long)(javg % 10U),
               (unsigned long)jover, (unsigned long)MitCtrl_Ticks());
    } else if (strcmp(arg, "reset") == 0) {
        MitCtrl_ResetJitter();
        printf("抖动统计已清零\r\n");
    } else {
        MitCtrl_GetJitter(&jmin, &jmax, &javg, &jover);
        printf("模式=%s kp=%.0f kd=%.1f drag_kd=%.2f online=%u enabled=%u\r\n",
               MitCtrl_ModeName(MitCtrl_Mode()), MitCtrl_Kp(), MitCtrl_Kd(),
               MitCtrl_DragKd(),
               (unsigned)MotorMgr_OnlineCount(), (unsigned)MotorMgr_EnabledCount());
        printf("周期: min=%lu max=%lu avg=%lu.%lums over=%lu ticks=%lu\r\n",
               (unsigned long)jmin, (unsigned long)jmax,
               (unsigned long)(javg / 10U), (unsigned long)(javg % 10U),
               (unsigned long)jover, (unsigned long)MitCtrl_Ticks());
        printf("用法: ctrl zero|hold|drag|kp <v>|kd <v>|dragkd [<n>] <v>|jitter|grav|reset\r\n");
    }
}

static void CanCommand(const char *arg)
{
    bsp_can_stats_t st;
    static const char *const kStateName[] = {"OK", "WARN", "PASSIVE", "BUSOFF"};

    if (strcmp(arg, "hw") == 0) {
        if (CAN_Bus_SetHardware(1)) {
            printf("CAN: 已切到真实双 CAN（1Mbit/s）\r\n");
        } else {
            printf("CAN: 初始化失败\r\n");
        }
    } else if (strcmp(arg, "sim") == 0) {
        (void)CAN_Bus_SetHardware(0);
        printf("CAN: 已切回虚拟总线\r\n");
    } else if (strcmp(arg, "loop") == 0) {
        if (CAN_Bus_SetLoopback(1)) {
            printf("CAN: 内部回环模式（无需第二个节点）\r\n");
        } else {
            printf("CAN: 切回环失败\r\n");
        }
    } else if (strcmp(arg, "normal") == 0) {
        if (CAN_Bus_SetLoopback(0)) {
            printf("CAN: 正常模式（总线上必须有另一个节点应答）\r\n");
        } else {
            printf("CAN: 切正常失败\r\n");
        }
    } else if (strcmp(arg, "sniff on") == 0) {
        MotorMgr_SetSniff(1);
        printf("CAN: 原始帧打印已打开（can sniff off 关闭）\r\n");
    } else if (strcmp(arg, "sniff off") == 0) {
        MotorMgr_SetSniff(0);
        printf("CAN: 原始帧打印已关闭\r\n");
    } else if (strcmp(arg, "cross") == 0) {
        static const uint8_t payload[8] = {0xCA, 0xFE, 0x00, 0x11,
                                           0x22, 0x33, 0x44, 0x55};
        rs_can_frame_t f;
        uint32_t rx0, rx1;
        bsp_can_stats_t s1;

        BSP_CAN_GetStats(BSP_CAN_1, &s1);
        rx0 = s1.rx_frames;
        BSP_CAN_GetStats(BSP_CAN_2, &s1);
        rx1 = s1.rx_frames;

        f.id  = 0x18FF0001U;
        f.dlc = 8;
        memcpy(f.data, payload, 8);
        printf("CAN1 发送 → 期望 CAN2 收到（需 CANH/CANL 并联 + 120Ω 终端）\r\n");
        (void)CAN_Bus_Send(0, &f);
        HAL_Delay(30);

        BSP_CAN_GetStats(BSP_CAN_1, &s1);
        printf("CAN1 rx +%lu  ", (unsigned long)(s1.rx_frames - rx0));
        BSP_CAN_GetStats(BSP_CAN_2, &s1);
        printf("CAN2 rx +%lu  (TEC=%u REC=%u)\r\n",
               (unsigned long)(s1.rx_frames - rx1), (unsigned)s1.tec, (unsigned)s1.rec);
    } else if (strcmp(arg, "test") == 0) {
        static const uint8_t payload[8] = {0x11, 0x22, 0x33, 0x44,
                                           0x55, 0x66, 0x77, 0x88};
        rs_can_frame_t f;
        uint32_t rx0, rx1;

        (void)CAN_Bus_SetHardware(1);
        (void)CAN_Bus_SetLoopback(1);
        BSP_CAN_GetStats(BSP_CAN_1, &st);
        rx0 = st.rx_frames;
        BSP_CAN_GetStats(BSP_CAN_2, &st);
        rx1 = st.rx_frames;

        f.id  = 0x12345678U;
        f.dlc = 8;
        memcpy(f.data, payload, 8);
        (void)CAN_Bus_Send(0, &f);
        (void)CAN_Bus_Send(1, &f);
        HAL_Delay(20);

        BSP_CAN_GetStats(BSP_CAN_1, &st);
        printf("CAN1: rx +%lu (总 %lu)，tx=%lu\r\n",
               (unsigned long)(st.rx_frames - rx0),
               (unsigned long)st.rx_frames, (unsigned long)st.tx_frames);
        BSP_CAN_GetStats(BSP_CAN_2, &st);
        printf("CAN2: rx +%lu (总 %lu)，tx=%lu\r\n",
               (unsigned long)(st.rx_frames - rx1),
               (unsigned long)st.rx_frames, (unsigned long)st.tx_frames);
        if ((st.rx_frames - rx1) >= 1U) {
            printf("回环自测：%s\r\n", "OK");
        }
    } else {
        for (uint8_t i = 0; i < BSP_CAN_COUNT; i++) {
            BSP_CAN_GetStats((bsp_can_id_t)i, &st);
            printf("CAN%u: %s loop=%u %s err=0x%02X TEC=%u REC=%u rx=%lu tx=%lu lost=%lu\r\n",
                   (unsigned)(i + 1U),
                   CAN_Bus_IsHardware() ? "hw" : "sim",
                   (unsigned)st.loopback,
                   (st.state == 0xFFU) ? "INIT" : kStateName[(st.state <= 3U) ? st.state : 3U],
                   (unsigned)st.last_error,
                   (unsigned)st.tec, (unsigned)st.rec,
                   (unsigned long)st.rx_frames,
                   (unsigned long)st.tx_frames,
                   (unsigned long)st.rx_lost);
        }
    }
}

static void SimCommand(const char *arg)
{
    char  sub[16];
    char  a1[16];
    char  a2[16];
    const char *p1;
    const char *p2;

    a1[0] = '\0';
    a2[0] = '\0';
    p1 = strchr(arg, ' ');
    p2 = (p1 != NULL) ? strchr(p1 + 1, ' ') : NULL;

    size_t n = p1 ? (size_t)(p1 - arg) : strlen(arg);
    if (n >= sizeof(sub)) { n = sizeof(sub) - 1U; }
    memcpy(sub, arg, n);
    sub[n] = '\0';

    if (p1 != NULL) {
        size_t n1 = p2 ? (size_t)(p2 - p1 - 1) : strlen(p1 + 1);
        if (n1 >= sizeof(a1)) { n1 = sizeof(a1) - 1U; }
        memcpy(a1, p1 + 1, n1);
        a1[n1] = '\0';
    }
    if (p2 != NULL) {
        snprintf(a2, sizeof(a2), "%s", p2 + 1);
    }

    if (strcmp(sub, "move") == 0) {
        int id = atoi(a1);
        double deg = atof(a2);
        Sim_MoveTarget((uint8_t)id, deg / 57.29578);
        {
            int d10 = (int)(deg * 10.0 + (deg >= 0 ? 0.5 : -0.5));
            printf("sim: 电机 %d 目标 %d.%d°\r\n", id, d10 / 10,
                   (int)((d10 % 10 < 0) ? -(d10 % 10) : (d10 % 10)));
        }
    } else if (strcmp(sub, "fault") == 0) {
        int id = atoi(a1);
        int bits = (int)strtol(a2, NULL, 0);
        Sim_SetFault((uint8_t)id, (uint8_t)bits);
        printf("sim: 电机 %d 故障位 0x%02X\r\n", id, bits);
    } else if (strcmp(sub, "offline") == 0) {
        int id = atoi(a1);
        Sim_SetOffline((uint8_t)id, atoi(a2) != 0);
        printf("sim: 电机 %d %s\r\n", id, atoi(a2) ? "掉线" : "恢复");
    } else {
        printf("用法: sim move <id> <角度> | sim fault <id> <bits> | sim offline <id> <0/1>\r\n");
    }
}

static void LcdCommand(const char *arg)
{
    if (strcmp(arg, "init") == 0) {
        BSP_LCD_Init();
        printf("LCD: 初始化完成 1.54\" ST7789 %ux%u（SPI2 21MHz）\r\n",
               (unsigned)BSP_LCD_Width(), (unsigned)BSP_LCD_Height());
    } else if (strcmp(arg, "info") == 0) {
        printf("LCD ready=%d %ux%u dir=%u BL(GPIO)=%d\r\n",
               BSP_LCD_IsReady(), (unsigned)BSP_LCD_Width(),
               (unsigned)BSP_LCD_Height(),
               (unsigned)(BSP_LCD_Width() > BSP_LCD_Height() ? 1U : 0U),
               (int)HAL_GPIO_ReadPin(BOARD_LCD_BL_PORT, BOARD_LCD_BL_PIN));
    } else if (strcmp(arg, "dir h") == 0) {
        BSP_LCD_SetDirection(BSP_LCD_DIR_LANDSCAPE);
        printf("LCD: 横屏 %ux%u\r\n",
               (unsigned)BSP_LCD_Width(), (unsigned)BSP_LCD_Height());
    } else if (strcmp(arg, "dir v") == 0) {
        BSP_LCD_SetDirection(BSP_LCD_DIR_PORTRAIT);
        printf("LCD: 竖屏 %ux%u\r\n",
               (unsigned)BSP_LCD_Width(), (unsigned)BSP_LCD_Height());
    } else if (strcmp(arg, "dir v180") == 0) {
        BSP_LCD_SetDirection(BSP_LCD_DIR_PORTRAIT_180);
        printf("LCD: 竖屏 180° %ux%u\r\n",
               (unsigned)BSP_LCD_Width(), (unsigned)BSP_LCD_Height());
    } else if (strcmp(arg, "dir h180") == 0) {
        BSP_LCD_SetDirection(BSP_LCD_DIR_LANDSCAPE_180);
        printf("LCD: 横屏 180° %ux%u\r\n",
               (unsigned)BSP_LCD_Width(), (unsigned)BSP_LCD_Height());
    } else if (strcmp(arg, "sniff on") == 0) {
        MotorMgr_SetSniff(1);
        printf("CAN: 原始帧打印已打开（can sniff off 关闭）\r\n");
    } else if (strcmp(arg, "sniff off") == 0) {
        MotorMgr_SetSniff(0);
        printf("CAN: 原始帧打印已关闭\r\n");
    } else if (strcmp(arg, "cross") == 0) {
        static const uint8_t payload[8] = {0xCA, 0xFE, 0x00, 0x11,
                                           0x22, 0x33, 0x44, 0x55};
        rs_can_frame_t f;
        uint32_t rx0, rx1;
        bsp_can_stats_t s1;

        BSP_CAN_GetStats(BSP_CAN_1, &s1);
        rx0 = s1.rx_frames;
        BSP_CAN_GetStats(BSP_CAN_2, &s1);
        rx1 = s1.rx_frames;

        f.id  = 0x18FF0001U;
        f.dlc = 8;
        memcpy(f.data, payload, 8);
        printf("CAN1 发送 → 期望 CAN2 收到（需 CANH/CANL 并联 + 120Ω 终端）\r\n");
        (void)CAN_Bus_Send(0, &f);
        HAL_Delay(30);

        BSP_CAN_GetStats(BSP_CAN_1, &s1);
        printf("CAN1 rx +%lu  ", (unsigned long)(s1.rx_frames - rx0));
        BSP_CAN_GetStats(BSP_CAN_2, &s1);
        printf("CAN2 rx +%lu  (TEC=%u REC=%u)\r\n",
               (unsigned long)(s1.rx_frames - rx1), (unsigned)s1.tec, (unsigned)s1.rec);
    } else if (strcmp(arg, "test") == 0) {
        BSP_LCD_TestPattern();
        printf("LCD: 已显示红/绿/蓝三色条\r\n");
    } else if (strcmp(arg, "on") == 0) {
        BSP_LCD_Backlight(1);
        printf("LCD: 背光 ON\r\n");
    } else if (strcmp(arg, "off") == 0) {
        BSP_LCD_Backlight(0);
        printf("LCD: 背光 OFF\r\n");
    } else if (strncmp(arg, "fill ", 5) == 0) {
        unsigned long rgb = strtoul(arg + 5, NULL, 16);
        uint16_t color = (uint16_t)((((rgb >> 16) & 0xF8U) << 8) |
                                    (((rgb >> 8) & 0xFCU) << 3) |
                                    ((rgb & 0xF8U) >> 3));
        BSP_LCD_Fill(color);
        BSP_LCD_Backlight(1);
        printf("LCD: 填充 0x%04X\r\n", color);
    } else {
        printf("用法: lcd init|info|test|on|off|dir v|v180|h|h180|fill <RRGGBB>\r\n");
    }
}

static void UiScreenCommand(const char *arg)
{
    static const struct {
        const char *name;
        ui_screen_t screen;
    } kMap[] = {
        {"main", UI_SCREEN_MAIN}, {"rec", UI_SCREEN_REC},
        {"play", UI_SCREEN_CSP},  {"mode", UI_SCREEN_MIT},
        {"zero", UI_SCREEN_ZERO}, {"info", UI_SCREEN_INFO},
    };

    if (strcmp(arg, "text") == 0) {
        UI_Text_Register();
        printf("UI 后端 = 串口文本\r\n");
        UI_Render();
        return;
    }
    if (strcmp(arg, "lcd") == 0) {
        UI_LCD_Register();
        printf("UI 后端 = 屏幕\r\n");
        UI_Render();
        return;
    }
    for (unsigned i = 0; i < sizeof(kMap) / sizeof(kMap[0]); i++) {
        if (strcmp(arg, kMap[i].name) == 0) {
            UI_SetScreen(kMap[i].screen);
            return;
        }
    }
    printf("用法: ui [main|rec|play|mode|zero|info|text|lcd]\r\n");
}

static void HandleLine(char *line)
{
    if (strcmp(line, "help") == 0) {
        printf("命令列表:\r\n");
        printf("  help                       显示帮助\r\n");
        printf("  ping                       回 pong\r\n");
        printf("  echo <文本>                 原样返回\r\n");
        printf("  clk                        打印时钟配置\r\n");
        printf("  id                         读芯片 ID / Flash 容量\r\n");
        printf("  led <can|fault> <on|off|toggle>\r\n");
        printf("  key <up|down|left|right> [long]   注入虚拟按键\r\n");
        printf("  keys                       打印当前按键电平\r\n");
        printf("  ui [页名]                  重绘/跳转到指定页（main/mit/csp/rec/zero/set/info/safe）\r\n");
        printf("  lcd init|info|test|on|off|blinv|regs|speed <n>|mode <0|3>|dir h|v|h180|v180|fill <RRGGBB>\r\n");
        printf("  motor discover/adopt/scan/poll/on/off/zero/hold/status/move\r\n");
        printf("  can [hw|sim|loop|normal|test|cross|sniff on/off]  双 CAN（CAN1=PB8/9，CAN2=PB12/6）\r\n");
        printf("  ctrl [zero|hold|drag|kp <v>|kd <v>|jitter|reset]  MIT 控制（100Hz）\r\n");
        printf("  teach <槽位> | rec/rec stop | play <槽位> [次数(0=无限)] [间隔ms] | stop\r\n");
        printf("  traj | traj dump <槽位> | traj mode csp|mit | traj smooth <ms> | flash\r\n");
        printf("  sim move <id> <角度> | sim fault <id> <bits> | sim offline <id> <0/1>\r\n");
        printf("  reset                      软复位\r\n");
    } else if (strcmp(line, "ping") == 0) {
        printf("pong\r\n");
    } else if (strncmp(line, "echo ", 5) == 0) {
        printf("%s\r\n", line + 5);
    } else if (strcmp(line, "clk") == 0) {
        UartPrintClock();
    } else if (strcmp(line, "id") == 0) {
        uint32_t idcode = *(volatile uint32_t *)0xE0042000U;
        uint16_t flash_kb = *(volatile uint16_t *)0x1FFF7A22U;
        printf("DBGMCU_IDCODE = 0x%08lx  DEV_ID = 0x%03lx  REV_ID = 0x%04lx\r\n",
               (unsigned long)idcode,
               (unsigned long)(idcode & 0xFFFU),
               (unsigned long)((idcode >> 16) & 0xFFFFU));
        printf("Flash = %u KB\r\n", (unsigned)flash_kb);
    } else if (strcmp(line, "keys") == 0) {
        for (uint8_t k = 0; k < (uint8_t)BSP_KEY_COUNT; k++) {
            if (BSP_Key_IsEnabled((bsp_key_t)k)) {
                printf("  KEY %-5s = %d\r\n", BSP_Key_Name((bsp_key_t)k),
                       BSP_Key_IsDown((bsp_key_t)k));
            } else {
                printf("  KEY %-5s = (未接，用 key 命令注入)\r\n",
                       BSP_Key_Name((bsp_key_t)k));
            }
        }
    } else if (strncmp(line, "led ", 4) == 0) {
        LedCommand(line + 4);
    } else if (strncmp(line, "key ", 4) == 0) {
        KeyCommand(line + 4);
    } else if (strncmp(line, "motor ", 6) == 0) {
        MotorCommand(line + 6);
    } else if (strncmp(line, "rec", 3) == 0) {
        char a1[16] = {0};
        const char *sp = strchr(line, ' ');

        if (sp != NULL) {
            snprintf(a1, sizeof(a1), "%s", sp + 1);
        }
        if (strcmp(a1, "stop") == 0) {
            Traj_StopRecord();
        } else {
            int slot = (a1[0] != '\0') ? atoi(a1) : 1;
            if (slot < 1 || slot > TRAJ_SLOTS) {
                slot = 1;
            }
            if (Traj_StartRecord((uint8_t)(slot - 1)) != 0) {
                printf("录制启动失败\r\n");
            }
        }
    } else if (strncmp(line, "teach", 5) == 0) {
        int slot = atoi(line + 5);
        if (slot < 1 || slot > TRAJ_SLOTS) {
            slot = 1;
        }
        if (Traj_StartRecord((uint8_t)(slot - 1)) != 0) {
            printf("示教录制启动失败\r\n");
        }
    } else if (strncmp(line, "play ", 5) == 0) {
        int  slot  = atoi(line + 5);
        char *sp   = strchr(line + 5, ' ');
        int  loops = 1;
        int  gap   = -1;

        if (sp != NULL) {
            loops = atoi(sp + 1);
            {
                char *sp2 = strchr(sp + 1, ' ');
                if (sp2 != NULL) {
                    gap = atoi(sp2 + 1);
                }
            }
        }
        if (gap >= 0) {
            Traj_SetPlayGap((uint16_t)gap);
        }

        if (slot >= 1 && slot <= TRAJ_SLOTS) {
            if (Traj_StartPlay((uint8_t)(slot - 1),
                               (uint8_t)((loops < 0) ? 0 : loops)) != 0) {
                printf("回放启动失败\r\n");
            }
        } else {
            printf("用法: play <1..5> [次数(0=无限)] [间隔ms]\r\n");
        }
    } else if (strncmp(line, "traj mode", 9) == 0) {
        const char *m = line + 9;
        while (*m == ' ') { m++; }
        if (strcmp(m, "mit") == 0) {
            Traj_SetPlayMode(1);
            printf("回放方式 = MIT（位置跟踪，kp=%.0f kd=%.2f，ctrl kp/kd 可调）\r\n",
                   MitCtrl_Kp(), MitCtrl_Kd());
        } else if (strcmp(m, "csp") == 0) {
            Traj_SetPlayMode(0);
            printf("回放方式 = CSP（loc_ref）\r\n");
        } else {
            printf("当前回放方式 = %s（用 traj mode csp|mit 切换）\r\n",
                   Traj_GetPlayMode() ? "MIT" : "CSP");
        }
    } else if (strncmp(line, "traj smooth", 11) == 0) {
        int tau = atoi(line + 11);
        Traj_SetSmooth((uint16_t)((tau < 0) ? 0 : tau));
        printf("回放平滑 τ = %u ms%s\r\n", (unsigned)Traj_GetSmooth(),
               (Traj_GetSmooth() == 0U) ? "（已关闭）" : "");
    } else if (strncmp(line, "traj save", 9) == 0) {
        int slot = atoi(line + 9);
        if (slot < 1 || slot > TRAJ_SLOTS) {
            slot = 1;
        }
        (void)Traj_SaveToFlash((uint8_t)(slot - 1));
    } else if (strncmp(line, "traj load", 9) == 0) {
        int slot = atoi(line + 9);
        if (slot < 1 || slot > TRAJ_SLOTS) {
            slot = 1;
        }
        if (Traj_LoadFromFlash((uint8_t)(slot - 1)) == 0) {
            printf("槽位 %d 已从 Flash 载入\r\n", slot);
        } else {
            printf("槽位 %d 载入失败（空/CRC 错）\r\n", slot);
        }
    } else if (strcmp(line, "flash") == 0) {
        printf("W25Q64 JEDEC=0x%06lX ready=%d\r\n",
               (unsigned long)W25Q_JedecId(), W25Q_IsReady());
    } else if (strncmp(line, "traj dump", 9) == 0) {
        int slot = atoi(line + 9);
        const traj_meta_t *mt;

        if (slot < 1 || slot > TRAJ_SLOTS) {
            slot = 1;
        }
        mt = Traj_GetMeta((uint8_t)(slot - 1));
        if (mt == NULL || !mt->valid) {
            printf("槽位 %d 无有效轨迹\r\n", slot);
        } else {
            printf("槽位 %d: %u 帧，各轴首帧→末帧（°）\r\n", slot,
                   (unsigned)mt->frames);
            for (uint8_t a = 0; a < TRAJ_AXES; a++) {
                int16_t v0 = Traj_GetSample((uint8_t)(slot - 1), 0, a);
                int16_t v1 = Traj_GetSample((uint8_t)(slot - 1),
                                            (uint16_t)(mt->frames - 1U), a);
                {
                    int d0 = (int)((double)v0 / TRAJ_SCALE * 57.29578 * 10.0);
                    int d1 = (int)((double)v1 / TRAJ_SCALE * 57.29578 * 10.0);
                    printf("  #%2u  %6d -> %6d  (%d.%d° -> %d.%d°)\r\n",
                           (unsigned)(a + 1U), (int)v0, (int)v1,
                           d0 / 10, (d0 % 10 < 0) ? -(d0 % 10) : (d0 % 10),
                           d1 / 10, (d1 % 10 < 0) ? -(d1 % 10) : (d1 % 10));
                }
            }
        }
    } else if (strcmp(line, "stop") == 0) {
        Traj_StopRecord();
        Traj_StopPlay();
        printf("已停止录制/回放\r\n");
    } else if (strcmp(line, "traj") == 0) {
        for (uint8_t i = 0; i < TRAJ_SLOTS; i++) {
            const traj_meta_t *mt = Traj_GetMeta(i);
            printf("  槽%u: %s  %u 帧  %u ms\r\n", (unsigned)(i + 1U),
                   (mt != NULL && mt->valid) ? "有效" : "空  ",
                   (unsigned)((mt != NULL) ? mt->frames : 0U),
                   (unsigned)((mt != NULL) ? mt->duration_ms : 0U));
        }
        printf("录制中=%u 回放中=%u\r\n",
               (unsigned)Traj_Recording(), (unsigned)Traj_Playing());
    } else if (strncmp(line, "ctrl ", 5) == 0) {
        CtrlCommand(line + 5);
    } else if (strcmp(line, "ctrl") == 0) {
        CtrlCommand("");
    } else if (strncmp(line, "can ", 4) == 0) {
        CanCommand(line + 4);
    } else if (strcmp(line, "can") == 0) {
        CanCommand("");
    } else if (strncmp(line, "sim ", 4) == 0) {
        SimCommand(line + 4);
    } else if (strncmp(line, "lcd ", 4) == 0) {
        LcdCommand(line + 4);
    } else if (strcmp(line, "ui") == 0) {
        UI_Render();
    } else if (strncmp(line, "ui ", 3) == 0) {
        UiScreenCommand(line + 3);
    } else if (strcmp(line, "reset") == 0) {
        printf("rebooting...\r\n");
        HAL_Delay(20);
        NVIC_SystemReset();
    } else {
        printf("未知命令: %s（输入 help）\r\n", line);
    }
}

static void UartPrintClock(void)
{
    RCC_ClkInitTypeDef clk;
    uint32_t flash_latency = 0;

    HAL_RCC_GetClockConfig(&clk, &flash_latency);

    printf(" SYSCLK: %lu Hz  HCLK: %lu Hz\r\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(),
           (unsigned long)HAL_RCC_GetHCLKFreq());
    printf(" PCLK1 : %lu Hz  PCLK2: %lu Hz\r\n",
           (unsigned long)HAL_RCC_GetPCLK1Freq(),
           (unsigned long)HAL_RCC_GetPCLK2Freq());
    printf(" source: %s  flash latency: %lu\r\n",
           (clk.SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK) ? "PLL" : "other",
           (unsigned long)flash_latency);
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#ifdef HSI_ONLY_CLOCK
    /* 诊断模式：只用内部 HSI 16MHz，不碰 PLL（make EXTRA_DEFS=-DHSI_ONLY_CLOCK） */
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
    return;
#endif

#ifdef USE_HSE_CLOCK
    /* ---- 默认：HSE 25MHz + PLL -> 168MHz，晶振起不来则退回 HSI ---- */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = (HSE_VALUE / 1000000U);
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 7;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState            = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLState        = RCC_PLL_ON;
        osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM            = 16;
        osc.PLL.PLLN            = 336;
        osc.PLL.PLLP            = RCC_PLLP_DIV2;
        osc.PLL.PLLQ            = 7;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
            Error_Handler();
        }
    }
#else
    /* ---- make CLOCK=hsi：不依赖晶振 ---- */
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 16;
    osc.PLL.PLLN            = 336;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }
#endif

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        BSP_LED_Toggle(BSP_LED_CAN);
        BSP_LED_Toggle(BSP_LED_FAULT);
        for (volatile uint32_t i = 0; i < 400000U; i++) {
            __NOP();
        }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("Assert failed: %s:%lu\r\n", (char *)file, (unsigned long)line);
    Error_Handler();
}
#endif
