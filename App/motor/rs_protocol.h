/**
 * @file    rs_protocol.h
 * @brief   RobStride（灵足）私有协议 —— protocol.hpp 的 C 版
 *
 * 逐条对齐 /data/lite_motor_ws/src/lite_driver/include/lite_driver/protocol.hpp：
 *   · 29 位扩展 ID：[28:24] comm_type | [23:8] extra_data | [7:0] device_id
 *   · MIT 帧：type 1，torque 藏在 ID 的 extra_data，payload = pos|vel|kp|kd 大端 u16
 *   · 类型 2 应答 / 类型 24 主动上报共用 payload 布局
 *   · limit_cur(0x7018) 是峰值 Apk 口径，扭矩→电流必须 ×√2
 *
 * 本文件不依赖 HAL，可在 PC 上直接编译单测（见 Tests/test_protocol.c）。
 */
#ifndef __RS_PROTOCOL_H__
#define __RS_PROTOCOL_H__

#include <stddef.h>
#include <stdint.h>

#define RS_HOST_ID_DEFAULT      0xFDU
#define RS_MAX_PAYLOAD          8U

/* ---------------------------- 通信类型 ---------------------------- */
typedef enum {
    RS_COMM_GET_ID          = 0,
    RS_COMM_OP_CTRL         = 1,
    RS_COMM_OP_STATUS       = 2,
    RS_COMM_ENABLE          = 3,
    RS_COMM_DISABLE         = 4,
    RS_COMM_SET_ZERO        = 6,
    RS_COMM_SET_ID          = 7,
    RS_COMM_READ_PARAM      = 17,
    RS_COMM_WRITE_PARAM     = 18,
    RS_COMM_FAULT_REPORT    = 21,
    RS_COMM_SAVE_PARAMS     = 22,
    RS_COMM_SET_BAUDRATE    = 23,
    RS_COMM_ACTIVE_REPORT   = 24,
    RS_COMM_SET_PROTOCOL    = 25,
} rs_comm_t;

/* ---------------------------- 参数 ID ---------------------------- */
#define RS_PARAM_RUN_MODE       0x7005U     /* u8  */
#define RS_PARAM_TORQUE_LIMIT   0x700BU     /* f32 N·m */
#define RS_PARAM_LOC_REF        0x7016U     /* f32 rad  */
#define RS_PARAM_LIMIT_SPD      0x7017U     /* f32 rad/s */
#define RS_PARAM_CURRENT_LIMIT  0x7018U     /* f32 A（峰值 Apk） */
#define RS_PARAM_MECH_POS       0x7019U     /* f32 rad 只读 */
#define RS_PARAM_MECH_VEL       0x701BU     /* f32 rad/s 只读 */
#define RS_PARAM_ACC_RAD        0x7022U     /* f32 rad/s² */
#define RS_PARAM_ZERO_STA       0x7029U     /* u8  */
#define RS_PARAM_FAULT_STA      0x3022U     /* u32 只读 */

#define RS_RUN_MODE_MIT         0U
#define RS_RUN_MODE_CSP         5U

/* ---------------------------- 型号量程 ---------------------------- */
typedef struct {
    double position;    /* rad，对称 ± */
    double velocity;    /* rad/s */
    double torque;      /* N·m（= 峰值） */
    double kp_max;
    double kd_max;
} rs_limits_t;

typedef struct {
    double rated;       /* N·m，额定（持续） */
    double peak;        /* N·m，峰值（瞬时，与 limits.torque 同源） */
} rs_torque_spec_t;

/* 型号 "rs-00".."rs-06"；未知型号返回 NULL */
const rs_limits_t      *rs_limits_for_model(const char *model);
/* 本臂 CAN ID → 型号量程（11/21 rs-02，12..14/22..24 rs-00，
 * 15..18/25..28 rs-05；未知 ID 兜底 rs-02）。 */
const rs_limits_t      *rs_limits_for_can_id(uint8_t id);
const rs_torque_spec_t *rs_torque_spec_for_model(const char *model);
/* 转矩常数 kt（N·m/Arms）；未知型号返回 0 */
double                  rs_torque_constant_for_model(const char *model);

/* 保护力矩(N·m) → limit_cur 需要的峰值电流 Apk（含 ×√2，非法输入返回 0） */
double rs_csp_limit_current_from_torque(double torque_nm, double kt_nm_per_arms);

/* ---------------------------- 帧结构 ---------------------------- */
typedef struct {
    uint32_t id;                    /* 29 位扩展 ID */
    uint8_t  dlc;
    uint8_t  bus;                   /* 0 = CAN1，1 = CAN2（虚拟总线恒为 0） */
    uint8_t  data[RS_MAX_PAYLOAD];
} rs_can_frame_t;

typedef struct {
    rs_comm_t comm_type;
    uint16_t  extra_data;
    uint8_t   device_id;
    const rs_can_frame_t *frame;
} rs_parsed_frame_t;

uint32_t rs_make_id(rs_comm_t type, uint16_t extra_data, uint8_t device_id);
int      rs_parse_frame(const rs_can_frame_t *f, rs_parsed_frame_t *out);
int      rs_is_status_like(rs_comm_t t);

/* ---------------------------- 量化 ---------------------------- */
uint16_t rs_scale_signed_u16(double value, double limit);
uint16_t rs_scale_unsigned_u16(double value, double limit);
double   rs_unscale_signed_u16(uint16_t raw, double limit);

/* ---------------------------- 编码 ---------------------------- */
void rs_encode_mit(rs_can_frame_t *f, uint8_t device_id, const rs_limits_t *lim,
                   double position, double velocity, double kp, double kd,
                   double torque);
void rs_encode_control(rs_can_frame_t *f, rs_comm_t type, uint8_t device_id,
                       uint8_t host_id, const uint8_t *payload, uint8_t dlc);
void rs_encode_write_param_float(rs_can_frame_t *f, uint8_t device_id,
                                 uint8_t host_id, uint16_t param_id, float value);
void rs_encode_write_param_u8(rs_can_frame_t *f, uint8_t device_id,
                              uint8_t host_id, uint16_t param_id, uint8_t value);
void rs_encode_read_param(rs_can_frame_t *f, uint8_t device_id,
                          uint8_t host_id, uint16_t param_id);
void rs_encode_save_params(rs_can_frame_t *f, uint8_t device_id, uint8_t host_id);
void rs_encode_active_report(rs_can_frame_t *f, uint8_t device_id,
                             uint8_t host_id, int on);

/* ---------------------------- 解码 ---------------------------- */
typedef struct {
    uint8_t device_id;
    double  position;       /* rad */
    double  velocity;       /* rad/s */
    double  torque;         /* N·m 反馈 */
    double  temperature;    /* degC */
    uint8_t fault_bits;     /* 概念位 0..5：欠压/栅驱/过温/编码器/堵转/未标定 */
} rs_status_t;

/** @return 1 成功；0 不是状态帧或长度不足 */
int rs_decode_status(const rs_parsed_frame_t *parsed, const rs_limits_t *lim,
                     rs_status_t *out);

typedef struct {
    uint16_t param_id;
    uint8_t  value[4];
} rs_read_reply_t;

/** @return 1 成功；0 类型不符或长度不足 */
int      rs_decode_read_param(const rs_parsed_frame_t *parsed, rs_read_reply_t *out);
uint32_t rs_read_param_u32(const rs_read_reply_t *r);
float    rs_read_param_f32(const rs_read_reply_t *r);
uint8_t  rs_read_param_u8(const rs_read_reply_t *r);

/**
 * 把 fault_bits 解成可读串（"过温|堵转过载"），0 返回空串。
 * @param buf 输出缓冲；@param n 缓冲长度
 * @return 传入的 buf
 */
const char *rs_describe_fault_bits(uint8_t bits, char *buf, size_t n);

#endif /* __RS_PROTOCOL_H__ */
