/**
 * @file    rs_protocol.c
 * @brief   RobStride 协议 C 实现（对齐 lite_driver/protocol.hpp）
 */
#include "rs_protocol.h"

#include <math.h>
#include <string.h>

/* ===================== 型号表（来源：灵足 RS 规格书） ===================== */
typedef struct {
    const char     *model;
    rs_limits_t     limits;
    rs_torque_spec_t torque;
    double          kt;     /* N·m/Arms */
} rs_model_row_t;

static const rs_model_row_t kModels[] = {
    {"rs-00", {4.0 * 3.14159265358979323846, 33.0, 14.0, 500.0, 5.0},   {5.0, 14.0}, 1.48},
    {"rs-01", {4.0 * 3.14159265358979323846, 44.0, 17.0, 500.0, 5.0},   {7.0, 17.0}, 1.22},
    {"rs-02", {4.0 * 3.14159265358979323846, 44.0, 17.0, 500.0, 5.0},   {7.0, 17.0}, 1.22},
    {"rs-03", {4.0 * 3.14159265358979323846, 20.0, 60.0, 5000.0, 100.0}, {21.0, 60.0}, 2.36},
    {"rs-04", {4.0 * 3.14159265358979323846, 15.0, 120.0, 5000.0, 100.0}, {35.0, 120.0}, 2.10},
    {"rs-05", {4.0 * 3.14159265358979323846, 50.0, 5.5, 500.0, 5.0},    {1.8, 5.5}, 0.94},
    {"rs-06", {4.0 * 3.14159265358979323846, 50.0, 36.0, 5000.0, 100.0}, {11.0, 36.0}, 1.10},
};

#define RS_MODEL_COUNT (sizeof(kModels) / sizeof(kModels[0]))

static const rs_model_row_t *find_model(const char *model)
{
    size_t i;

    if (model == NULL) {
        return NULL;
    }
    for (i = 0; i < RS_MODEL_COUNT; i++) {
        if (strcmp(model, kModels[i].model) == 0) {
            return &kModels[i];
        }
    }
    return NULL;
}

const rs_limits_t *rs_limits_for_model(const char *model)
{
    const rs_model_row_t *row = find_model(model);
    return row ? &row->limits : NULL;
}

/* 本臂实测型号（joint_spec.yaml，CAN ID 固定）：
 *   11/21 shoulder_pitch rs-02；12..14/22..24 rs-00；15..18/25..28 rs-05。
 * MIT 力矩/速度的 16 位量化必须按各自型号量程，否则 rs-05 轴会被放大/缩小
 * 17/5.5 ≈ 3.09 倍（重力前馈最容易暴露）。 */
const rs_limits_t *rs_limits_for_can_id(uint8_t id)
{
    switch (id) {
        case 11: case 21:
            return rs_limits_for_model("rs-02");
        case 12: case 13: case 14:
        case 22: case 23: case 24:
            return rs_limits_for_model("rs-00");
        case 15: case 16: case 17: case 18:
        case 25: case 26: case 27: case 28:
            return rs_limits_for_model("rs-05");
        default:
            return rs_limits_for_model("rs-02");
    }
}

const rs_torque_spec_t *rs_torque_spec_for_model(const char *model)
{
    const rs_model_row_t *row = find_model(model);
    return row ? &row->torque : NULL;
}

double rs_torque_constant_for_model(const char *model)
{
    const rs_model_row_t *row = find_model(model);
    return row ? row->kt : 0.0;
}

double rs_csp_limit_current_from_torque(double torque_nm, double kt_nm_per_arms)
{
    /* kt 是 N·m/Arms，而 0x7018 按峰值 Apk 解释 ⇒ ×√2。
     * 漏掉它实际力矩只有 70.7%（真机实测过，见 protocol.hpp 注释）。 */
    if (!(torque_nm > 0.0) || !(kt_nm_per_arms > 0.0)) {
        return 0.0;
    }
    return torque_nm / kt_nm_per_arms * 1.4142135623730951;
}

/* ===================== ID / 解析 ===================== */
uint32_t rs_make_id(rs_comm_t type, uint16_t extra_data, uint8_t device_id)
{
    return ((uint32_t)type << 24) |
           ((uint32_t)extra_data << 8) |
           (uint32_t)device_id;
}

int rs_parse_frame(const rs_can_frame_t *f, rs_parsed_frame_t *out)
{
    uint32_t raw;

    if (f == NULL || out == NULL) {
        return 0;
    }
    if (f->id > 0x1FFFFFFFU) {          /* 标准帧（11 位）直接拒绝 */
        return 0;
    }

    raw = (f->id >> 24) & 0x1FU;
    switch (raw) {                      /* kValidCommTypes 的等价展开 */
        case RS_COMM_GET_ID:
        case RS_COMM_OP_CTRL:
        case RS_COMM_OP_STATUS:
        case RS_COMM_ENABLE:
        case RS_COMM_DISABLE:
        case RS_COMM_SET_ZERO:
        case RS_COMM_SET_ID:
        case RS_COMM_READ_PARAM:
        case RS_COMM_WRITE_PARAM:
        case RS_COMM_FAULT_REPORT:
        case RS_COMM_SAVE_PARAMS:
        case RS_COMM_SET_BAUDRATE:
        case RS_COMM_ACTIVE_REPORT:
        case RS_COMM_SET_PROTOCOL:
            break;
        default:
            return 0;
    }

    out->comm_type  = (rs_comm_t)raw;
    out->extra_data = (uint16_t)((f->id >> 8) & 0xFFFFU);
    out->device_id  = (uint8_t)(f->id & 0xFFU);
    out->frame      = f;
    return 1;
}

int rs_is_status_like(rs_comm_t t)
{
    return (t == RS_COMM_OP_STATUS) || (t == RS_COMM_ACTIVE_REPORT);
}

/* ===================== 量化 ===================== */
uint16_t rs_scale_signed_u16(double value, double limit)
{
    double scaled;

    if (!(limit > 0.0)) {
        return 0;
    }
    if (value < -limit) {
        value = -limit;
    }
    if (value > limit) {
        value = limit;
    }
    scaled = ((value / limit) + 1.0) * 32767.0;
    if (!(scaled > 0.0)) {
        return 0;
    }
    if (scaled >= 65535.0) {
        return 65535U;
    }
    return (uint16_t)scaled;
}

uint16_t rs_scale_unsigned_u16(double value, double limit)
{
    double scaled;

    if (!(limit > 0.0)) {
        return 0;
    }
    if (value < 0.0) {
        value = 0.0;
    }
    if (value > limit) {
        value = limit;
    }
    scaled = (value / limit) * 65535.0;
    if (!(scaled > 0.0)) {
        return 0;
    }
    if (scaled >= 65535.0) {
        return 65535U;
    }
    return (uint16_t)scaled;
}

double rs_unscale_signed_u16(uint16_t raw, double limit)
{
    return (((double)raw / 32767.0) - 1.0) * limit;
}

/* ===================== 编码 ===================== */
static void put_be_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFU);
}

static uint16_t get_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void rs_encode_mit(rs_can_frame_t *f, uint8_t device_id, const rs_limits_t *lim,
                   double position, double velocity, double kp, double kd,
                   double torque)
{
    uint16_t pos_u, vel_u, kp_u, kd_u, trq_u;

    if (f == NULL || lim == NULL) {
        return;
    }

    pos_u = rs_scale_signed_u16(position, lim->position);
    vel_u = rs_scale_signed_u16(velocity, lim->velocity);
    trq_u = rs_scale_signed_u16(torque, lim->torque);
    kp_u  = rs_scale_unsigned_u16(kp, lim->kp_max);
    kd_u  = rs_scale_unsigned_u16(kd, lim->kd_max);

    f->id  = rs_make_id(RS_COMM_OP_CTRL, trq_u, device_id);
    f->dlc = 8;
    put_be_u16(&f->data[0], pos_u);
    put_be_u16(&f->data[2], vel_u);
    put_be_u16(&f->data[4], kp_u);
    put_be_u16(&f->data[6], kd_u);
}

void rs_encode_control(rs_can_frame_t *f, rs_comm_t type, uint8_t device_id,
                       uint8_t host_id, const uint8_t *payload, uint8_t dlc)
{
    uint8_t i;

    if (f == NULL) {
        return;
    }

    f->id  = rs_make_id(type, (uint16_t)host_id, device_id);
    f->dlc = (dlc > RS_MAX_PAYLOAD) ? RS_MAX_PAYLOAD : dlc;

    for (i = 0; i < RS_MAX_PAYLOAD; i++) {
        f->data[i] = 0;
    }
    if (payload != NULL && dlc > 0U) {
        for (i = 0; i < f->dlc; i++) {
            f->data[i] = payload[i];
        }
    }
}

void rs_encode_write_param_float(rs_can_frame_t *f, uint8_t device_id,
                                 uint8_t host_id, uint16_t param_id, float value)
{
    uint32_t bits;
    uint8_t  i;

    if (f == NULL) {
        return;
    }

    memcpy(&bits, &value, sizeof(bits));

    f->id  = rs_make_id(RS_COMM_WRITE_PARAM, (uint16_t)host_id, device_id);
    f->dlc = 8;
    f->data[0] = (uint8_t)(param_id & 0xFFU);
    f->data[1] = (uint8_t)((param_id >> 8) & 0xFFU);
    f->data[2] = 0;
    f->data[3] = 0;
    for (i = 0; i < 4; i++) {
        f->data[4 + i] = (uint8_t)((bits >> (8U * i)) & 0xFFU);
    }
}

void rs_encode_write_param_u8(rs_can_frame_t *f, uint8_t device_id,
                              uint8_t host_id, uint16_t param_id, uint8_t value)
{
    if (f == NULL) {
        return;
    }

    f->id  = rs_make_id(RS_COMM_WRITE_PARAM, (uint16_t)host_id, device_id);
    f->dlc = 8;
    f->data[0] = (uint8_t)(param_id & 0xFFU);
    f->data[1] = (uint8_t)((param_id >> 8) & 0xFFU);
    f->data[2] = 0;
    f->data[3] = 0;
    f->data[4] = value;
    f->data[5] = 0;
    f->data[6] = 0;
    f->data[7] = 0;
}

void rs_encode_read_param(rs_can_frame_t *f, uint8_t device_id,
                          uint8_t host_id, uint16_t param_id)
{
    uint8_t i;

    if (f == NULL) {
        return;
    }

    f->id  = rs_make_id(RS_COMM_READ_PARAM, (uint16_t)host_id, device_id);
    f->dlc = 8;
    f->data[0] = (uint8_t)(param_id & 0xFFU);
    f->data[1] = (uint8_t)((param_id >> 8) & 0xFFU);
    for (i = 2; i < 8; i++) {
        f->data[i] = 0;
    }
}

void rs_encode_save_params(rs_can_frame_t *f, uint8_t device_id, uint8_t host_id)
{
    uint8_t i;

    if (f == NULL) {
        return;
    }

    rs_encode_control(f, RS_COMM_SAVE_PARAMS, device_id, host_id, NULL, 0);
    f->dlc = 8;
    for (i = 0; i < 8; i++) {
        f->data[i] = (uint8_t)(i + 1U);     /* 固定 01..08 */
    }
}

void rs_encode_active_report(rs_can_frame_t *f, uint8_t device_id,
                             uint8_t host_id, int on)
{
    static const uint8_t kPrefix[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    uint8_t i;

    if (f == NULL) {
        return;
    }

    rs_encode_control(f, RS_COMM_ACTIVE_REPORT, device_id, host_id, NULL, 0);
    f->dlc = 8;
    for (i = 0; i < 6; i++) {
        f->data[i] = kPrefix[i];
    }
    f->data[6] = on ? 0x01U : 0x00U;        /* 使能字节在 data[6]，实测如此 */
    f->data[7] = 0x00;
}

/* ===================== 解码 ===================== */
int rs_decode_status(const rs_parsed_frame_t *parsed, const rs_limits_t *lim,
                     rs_status_t *out)
{
    const uint8_t *d;
    uint16_t pos_u, vel_u, trq_u, tmp_u;

    if (parsed == NULL || lim == NULL || out == NULL || parsed->frame == NULL) {
        return 0;
    }
    if (!rs_is_status_like(parsed->comm_type)) {
        return 0;
    }
    if (parsed->frame->dlc < 8U) {
        return 0;
    }

    d = parsed->frame->data;
    pos_u = get_be_u16(&d[0]);
    vel_u = get_be_u16(&d[2]);
    trq_u = get_be_u16(&d[4]);
    tmp_u = get_be_u16(&d[6]);

    out->device_id   = (uint8_t)(parsed->extra_data & 0xFFU);
    out->position    = rs_unscale_signed_u16(pos_u, lim->position);
    out->velocity    = rs_unscale_signed_u16(vel_u, lim->velocity);
    out->torque      = rs_unscale_signed_u16(trq_u, lim->torque);
    out->temperature = (double)tmp_u * 0.1;
    out->fault_bits  = (uint8_t)((parsed->extra_data >> 8) & 0x3FU);
    return 1;
}

int rs_decode_read_param(const rs_parsed_frame_t *parsed, rs_read_reply_t *out)
{
    const uint8_t *d;

    if (parsed == NULL || out == NULL || parsed->frame == NULL) {
        return 0;
    }
    if (parsed->comm_type != RS_COMM_READ_PARAM) {
        return 0;
    }
    if (parsed->frame->dlc < 8U) {
        return 0;
    }

    d = parsed->frame->data;
    out->param_id = (uint16_t)(((uint16_t)d[1] << 8) | d[0]);
    out->value[0] = d[4];
    out->value[1] = d[5];
    out->value[2] = d[6];
    out->value[3] = d[7];
    return 1;
}

uint32_t rs_read_param_u32(const rs_read_reply_t *r)
{
    return ((uint32_t)r->value[0]) |
           ((uint32_t)r->value[1] << 8) |
           ((uint32_t)r->value[2] << 16) |
           ((uint32_t)r->value[3] << 24);
}

float rs_read_param_f32(const rs_read_reply_t *r)
{
    uint32_t bits = rs_read_param_u32(r);
    float    v = 0.0F;

    memcpy(&v, &bits, sizeof(v));
    return v;
}

uint8_t rs_read_param_u8(const rs_read_reply_t *r)
{
    return r->value[0];
}

const char *rs_describe_fault_bits(uint8_t bits, char *buf, size_t n)
{
    static const struct {
        uint8_t     bit;
        const char *name;
    } kEntries[] = {
        {1U << 0, "欠压"},
        {1U << 1, "栅极驱动"},
        {1U << 2, "过温"},
        {1U << 3, "磁编码器"},
        {1U << 4, "堵转过载"},
        {1U << 5, "未标定"},
    };
    static const char kUnknown[] = "未知位";
    size_t  len = 0;
    size_t  i;
    size_t  k;
    uint8_t known = 0;

    if (buf == NULL || n == 0U) {
        return buf;
    }
    buf[0] = '\0';

    for (i = 0; i < sizeof(kEntries) / sizeof(kEntries[0]); i++) {
        if ((bits & kEntries[i].bit) == 0U) {
            continue;
        }
        known = (uint8_t)(known | kEntries[i].bit);
        if (len > 0U && len + 1U < n) {
            buf[len++] = '|';
        }
        for (k = 0; kEntries[i].name[k] != '\0' && len + 1U < n; k++) {
            buf[len++] = kEntries[i].name[k];
        }
    }

    if ((uint8_t)(bits & (uint8_t)~known) != 0U) {
        if (len > 0U && len + 1U < n) {
            buf[len++] = '|';
        }
        for (k = 0; kUnknown[k] != '\0' && len + 1U < n; k++) {
            buf[len++] = kUnknown[k];
        }
    }
    buf[len] = '\0';
    return buf;
}
