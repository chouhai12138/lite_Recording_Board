/**
 * @file    bsp_keys.c
 * @brief   按键扫描（消抖 + 长按 + 事件队列），支持每键独立极性与"未接"状态
 *
 * 当前硬件（SCH_STM32F4x_DAP_CORE_CAN485 原理图）：
 *   UP   = PA0  （KEYUP，接 3.3V，按下为高）
 *   DOWN = PD14 （KEY1，接 GND，按下为低）
 *   LEFT / RIGHT 在底板上，引脚未定 → 先禁用，用串口 `key left|right` 注入调试
 */
#include "bsp_keys.h"

#define KEY_DEBOUNCE_MS     20U
#define KEY_EVENT_QUEUE     16U

typedef struct {
    uint8_t       enabled;
    GPIO_TypeDef *port;
    uint16_t      pin;
    GPIO_PinState active;
} key_cfg_t;

typedef struct {
    uint8_t  stable;
    uint8_t  raw_last;
    uint32_t raw_change_ms;
    uint32_t press_ms;
    uint8_t  long_sent;
} key_state_t;

static const key_cfg_t s_cfg[BSP_KEY_COUNT] = {
    [BSP_KEY_UP] = {
        BOARD_KEY_UP_ENABLED, BOARD_KEY_UP_PORT, BOARD_KEY_UP_PIN,
        BOARD_KEY_UP_ACTIVE,
    },
#if BOARD_KEY_DOWN_ENABLED
    [BSP_KEY_DOWN] = {
        BOARD_KEY_DOWN_ENABLED, BOARD_KEY_DOWN_PORT, BOARD_KEY_DOWN_PIN,
        BOARD_KEY_DOWN_ACTIVE,
    },
#else
    [BSP_KEY_DOWN] = {0, NULL, 0, GPIO_PIN_RESET},
#endif
#if BOARD_KEY_LEFT_ENABLED
    [BSP_KEY_LEFT] = {
        BOARD_KEY_LEFT_ENABLED, BOARD_KEY_LEFT_PORT, BOARD_KEY_LEFT_PIN,
        BOARD_KEY_LEFT_ACTIVE,
    },
#else
    [BSP_KEY_LEFT] = {0, NULL, 0, GPIO_PIN_RESET},
#endif
#if BOARD_KEY_RIGHT_ENABLED
    [BSP_KEY_RIGHT] = {
        BOARD_KEY_RIGHT_ENABLED, BOARD_KEY_RIGHT_PORT, BOARD_KEY_RIGHT_PIN,
        BOARD_KEY_RIGHT_ACTIVE,
    },
#else
    [BSP_KEY_RIGHT] = {0, NULL, 0, GPIO_PIN_RESET},
#endif
};

static const char *const s_key_names[BSP_KEY_COUNT] = {
    [BSP_KEY_UP] = "UP", [BSP_KEY_DOWN] = "DOWN",
    [BSP_KEY_LEFT] = "LEFT", [BSP_KEY_RIGHT] = "RIGHT",
};

static key_state_t s_state[BSP_KEY_COUNT];

static bsp_key_event_t s_queue[KEY_EVENT_QUEUE];
static uint8_t         s_q_head;
static uint8_t         s_q_tail;

static uint8_t key_is_down_raw(bsp_key_t key)
{
    if (!s_cfg[key].enabled) {
        return 0;
    }
    return (HAL_GPIO_ReadPin(s_cfg[key].port, s_cfg[key].pin) == s_cfg[key].active)
           ? 1U : 0U;
}

static void queue_push(bsp_key_t key, bsp_key_ev_t ev, uint32_t hold_ms)
{
    uint8_t next = (uint8_t)((s_q_head + 1U) % KEY_EVENT_QUEUE);

    if (next == s_q_tail) {
        s_q_tail = (uint8_t)((s_q_tail + 1U) % KEY_EVENT_QUEUE);
    }
    s_queue[s_q_head].key     = key;
    s_queue[s_q_head].ev      = ev;
    s_queue[s_q_head].hold_ms = hold_ms;
    s_q_head = next;
}

void BSP_Keys_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    BOARD_KEYS_GPIO_CLK_ENABLE();

    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0; i < (uint8_t)BSP_KEY_COUNT; i++) {
        if (s_cfg[i].enabled) {
            /* 按下为低 → 上拉；按下为高 → 下拉 */
            gpio.Pull = (s_cfg[i].active == GPIO_PIN_RESET) ? GPIO_PULLUP
                                                            : GPIO_PULLDOWN;
            gpio.Pin = s_cfg[i].pin;
            HAL_GPIO_Init(s_cfg[i].port, &gpio);
        }
        s_state[i].stable        = 0;
        s_state[i].raw_last      = 0;
        s_state[i].raw_change_ms = 0;
        s_state[i].press_ms      = 0;
        s_state[i].long_sent      = 0;
    }
    s_q_head = 0;
    s_q_tail = 0;
}

void BSP_Keys_Scan(uint32_t now_ms)
{
    for (uint8_t i = 0; i < (uint8_t)BSP_KEY_COUNT; i++) {
        key_state_t *st  = &s_state[i];
        bsp_key_t    key = (bsp_key_t)i;
        uint8_t      raw;

        if (!s_cfg[i].enabled) {
            continue;
        }
        raw = key_is_down_raw(key);

        if (raw != st->raw_last) {
            st->raw_last      = raw;
            st->raw_change_ms = now_ms;
            continue;
        }

        if (st->stable == raw) {
            if (st->stable && !st->long_sent &&
                (now_ms - st->press_ms) >= BOARD_KEY_LONG_MS) {
                st->long_sent = 1U;
                queue_push(key, BSP_KEY_EV_LONG, now_ms - st->press_ms);
            }
            continue;
        }

        if ((now_ms - st->raw_change_ms) < KEY_DEBOUNCE_MS) {
            continue;
        }

        st->stable = raw;
        if (raw) {
            st->press_ms  = now_ms;
            st->long_sent = 0U;
            queue_push(key, BSP_KEY_EV_PRESS, 0U);
        } else {
            queue_push(key, BSP_KEY_EV_RELEASE, now_ms - st->press_ms);
        }
    }
}

int BSP_Keys_GetEvent(bsp_key_event_t *ev)
{
    if (s_q_tail == s_q_head) {
        return 0;
    }
    *ev = s_queue[s_q_tail];
    s_q_tail = (uint8_t)((s_q_tail + 1U) % KEY_EVENT_QUEUE);
    return 1;
}

int BSP_Key_IsDown(bsp_key_t key)
{
    if (key >= BSP_KEY_COUNT) {
        return 0;
    }
    return s_state[key].stable;
}

int BSP_Key_IsEnabled(bsp_key_t key)
{
    return (key < BSP_KEY_COUNT) ? s_cfg[key].enabled : 0;
}

void BSP_Keys_Inject(bsp_key_t key, int is_long)
{
    if (key >= BSP_KEY_COUNT) {
        return;
    }
    queue_push(key, BSP_KEY_EV_PRESS, 0U);
    if (is_long) {
        queue_push(key, BSP_KEY_EV_LONG, BOARD_KEY_LONG_MS);
    }
    queue_push(key, BSP_KEY_EV_RELEASE, is_long ? BOARD_KEY_LONG_MS : 0U);
}

const char *BSP_Key_Name(bsp_key_t key)
{
    return (key < BSP_KEY_COUNT) ? s_key_names[key] : "?";
}
