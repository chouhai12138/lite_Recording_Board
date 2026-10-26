/**
 * @file    test_protocol.c
 * @brief   rs_protocol 主机侧单测（对应 lite_driver/test/test_protocol.cpp）
 *
 * 逐条钉住位级布局与量化口径。主机编译运行：
 *     make test
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rs_protocol.h"

static int g_checks;
static int g_fails;

#define CHECK(cond)                                                       \
    do {                                                                  \
        g_checks++;                                                       \
        if (!(cond)) {                                                    \
            printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
            g_fails++;                                                    \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                             \
    do {                                                                  \
        double _a = (double)(a), _b = (double)(b);                        \
        g_checks++;                                                       \
        if (fabs(_a - _b) > (tol)) {                                      \
            printf("  FAIL  %s:%d  %s=%.9f != %s=%.9f (tol %g)\n",        \
                   __FILE__, __LINE__, #a, _a, #b, _b, (double)(tol));    \
            g_fails++;                                                    \
        }                                                                 \
    } while (0)

static rs_can_frame_t frame_with(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    rs_can_frame_t f;

    memset(&f, 0, sizeof(f));
    f.id = id;
    f.dlc = dlc;
    if (data != NULL) {
        memcpy(f.data, data, dlc > 8U ? 8U : dlc);
    }
    return f;
}

/* -------------------- 写参数（run_mode u8 / loc_ref f32） -------------------- */
static void test_write_param_u8_byte4(void)
{
    rs_can_frame_t  f;
    rs_parsed_frame_t p;

    printf("[test] write_param_u8 puts value in data[4]\n");
    rs_encode_write_param_u8(&f, 11, 0xFD, RS_PARAM_RUN_MODE, 5);

    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(p.comm_type == RS_COMM_WRITE_PARAM);
    CHECK(p.device_id == 11);
    CHECK((p.extra_data & 0xFF) == 0xFD);
    CHECK(f.dlc == 8);
    CHECK(f.data[0] == 0x05);
    CHECK(f.data[1] == 0x70);
    CHECK(f.data[2] == 0x00);
    CHECK(f.data[3] == 0x00);
    CHECK(f.data[4] == 5);
    CHECK(f.data[5] == 0x00);
    CHECK(f.data[6] == 0x00);
    CHECK(f.data[7] == 0x00);
}

static void test_write_param_float_loc_ref(void)
{
    rs_can_frame_t f;
    float back = 0.0F;

    printf("[test] write_param_float loc_ref layout\n");
    rs_encode_write_param_float(&f, 11, 0xFD, RS_PARAM_LOC_REF, -0.0303F);

    CHECK(f.data[0] == 0x16);
    CHECK(f.data[1] == 0x70);
    CHECK(f.data[2] == 0x00);
    CHECK(f.data[3] == 0x00);
    memcpy(&back, &f.data[4], sizeof(back));
    CHECK(fabsf(back - (-0.0303F)) < 1e-6F);
}

/* -------------------- 读参数（类型 17） -------------------- */
static void test_read_param_request(void)
{
    rs_can_frame_t f;

    printf("[test] read_param request carries index little-endian\n");
    rs_encode_read_param(&f, 11, 0xFD, RS_PARAM_MECH_POS);
    CHECK(f.dlc == 8);
    CHECK(f.data[0] == 0x19);
    CHECK(f.data[1] == 0x70);
    for (int i = 2; i < 8; i++) {
        CHECK(f.data[i] == 0x00);
    }
}

static void test_decode_read_param(void)
{
    const float value = -0.030333F;
    uint8_t bytes[4];
    uint8_t payload[8];
    rs_can_frame_t f;
    rs_parsed_frame_t p;
    rs_read_reply_t r;

    printf("[test] decode_read_param echoes index and returns value\n");
    memcpy(bytes, &value, sizeof(bytes));
    payload[0] = 0x19; payload[1] = 0x70; payload[2] = 0x00; payload[3] = 0x00;
    payload[4] = bytes[0]; payload[5] = bytes[1];
    payload[6] = bytes[2]; payload[7] = bytes[3];

    f = frame_with(rs_make_id(RS_COMM_READ_PARAM, 0x000B, 0xFD), payload, 8);
    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(rs_decode_read_param(&p, &r) == 1);
    CHECK(r.param_id == 0x7019);
    CHECK_NEAR(rs_read_param_f32(&r), value, 1e-6);
}

static void test_decode_read_param_rejects(void)
{
    uint8_t zero[8] = {0};
    rs_can_frame_t f;
    rs_parsed_frame_t p;
    rs_read_reply_t r;

    printf("[test] decode_read_param rejects wrong type / short payload\n");
    f = frame_with(rs_make_id(RS_COMM_OP_STATUS, 0x000B, 0xFD), zero, 8);
    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(rs_decode_read_param(&p, &r) == 0);

    f = frame_with(rs_make_id(RS_COMM_READ_PARAM, 0x000B, 0xFD), zero, 2);
    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(rs_decode_read_param(&p, &r) == 0);
}

static void test_read_param_scalars(void)
{
    rs_read_reply_t r;

    printf("[test] read_param scalar interpretation\n");
    memset(&r, 0, sizeof(r));
    r.param_id = RS_PARAM_RUN_MODE;
    r.value[0] = RS_RUN_MODE_CSP;
    CHECK(rs_read_param_u8(&r) == RS_RUN_MODE_CSP);
    CHECK(rs_read_param_u32(&r) == RS_RUN_MODE_CSP);

    r.value[0] = 0x00; r.value[1] = 0x00;
    r.value[2] = 0x80; r.value[3] = 0x3F;     /* 1.0f 小端 */
    CHECK(fabsf(rs_read_param_f32(&r) - 1.0F) < 1e-6F);
}

/* -------------------- 主动上报（类型 24） -------------------- */
static void test_active_report_byte6(void)
{
    rs_can_frame_t on, off;
    rs_parsed_frame_t p;

    printf("[test] active_report enable byte is data[6]\n");
    rs_encode_active_report(&on, 11, 0xFD, 1);
    CHECK(rs_parse_frame(&on, &p) == 1);
    CHECK(p.comm_type == RS_COMM_ACTIVE_REPORT);
    CHECK(p.device_id == 11);
    for (int i = 0; i < 6; i++) {
        CHECK(on.data[i] == (uint8_t)(i + 1));
    }
    CHECK(on.data[6] == 0x01);
    CHECK(on.data[7] == 0x00);

    rs_encode_active_report(&off, 11, 0xFD, 0);
    CHECK(off.data[6] == 0x00);
}

/* -------------------- 状态帧（类型 2 / 24） -------------------- */
static void test_status_like(void)
{
    printf("[test] is_status_like\n");
    CHECK(rs_is_status_like(RS_COMM_OP_STATUS) == 1);
    CHECK(rs_is_status_like(RS_COMM_ACTIVE_REPORT) == 1);
    CHECK(rs_is_status_like(RS_COMM_FAULT_REPORT) == 0);
    CHECK(rs_is_status_like(RS_COMM_READ_PARAM) == 0);
}

static void test_decode_status_active_report(void)
{
    const rs_limits_t *lim = rs_limits_for_model("rs-00");
    uint16_t pos_u, vel_u, trq_u, tmp_u = 250;
    uint8_t payload[8];
    rs_can_frame_t f;
    rs_parsed_frame_t p;
    rs_status_t s;

    printf("[test] decode_status accepts type-24 layout\n");
    CHECK(lim != NULL);
    pos_u = rs_scale_signed_u16(1.0, lim->position);
    vel_u = rs_scale_signed_u16(-2.0, lim->velocity);
    trq_u = rs_scale_signed_u16(0.5, lim->torque);

    payload[0] = (uint8_t)(pos_u >> 8); payload[1] = (uint8_t)(pos_u & 0xFF);
    payload[2] = (uint8_t)(vel_u >> 8); payload[3] = (uint8_t)(vel_u & 0xFF);
    payload[4] = (uint8_t)(trq_u >> 8); payload[5] = (uint8_t)(trq_u & 0xFF);
    payload[6] = (uint8_t)(tmp_u >> 8); payload[7] = (uint8_t)(tmp_u & 0xFF);

    f = frame_with(rs_make_id(RS_COMM_ACTIVE_REPORT, 0x000B, 0xFD), payload, 8);
    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(rs_decode_status(&p, lim, &s) == 1);
    CHECK(s.device_id == 11);
    CHECK_NEAR(s.position, 1.0, 0.01);
    CHECK_NEAR(s.velocity, -2.0, 0.01);
    CHECK_NEAR(s.torque, 0.5, 0.01);
    CHECK_NEAR(s.temperature, 25.0, 1e-9);
    CHECK(s.fault_bits == 0);
}

static void test_decode_status_rejects(void)
{
    uint8_t zero[8] = {0};
    rs_can_frame_t f;
    rs_parsed_frame_t p;
    rs_status_t s;
    const rs_limits_t *lim = rs_limits_for_model("rs-00");

    printf("[test] decode_status ignores non-status types\n");
    f = frame_with(rs_make_id(RS_COMM_FAULT_REPORT, 0x000B, 0xFD), zero, 8);
    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(rs_decode_status(&p, lim, &s) == 0);
}

static void test_status_fault_bits(void)
{
    uint8_t payload[8] = {0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0xFA};
    rs_can_frame_t f;
    rs_parsed_frame_t p;
    rs_status_t s;
    const rs_limits_t *lim = rs_limits_for_model("rs-00");

    printf("[test] fault bits come from extra high byte\n");
    f = frame_with(rs_make_id(RS_COMM_ACTIVE_REPORT, 0x020B, 0xFD), payload, 8);
    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(rs_decode_status(&p, lim, &s) == 1);
    CHECK(s.device_id == 11);
    CHECK(s.fault_bits == 0x02);
}

/* -------------------- MIT 帧 -------------------- */
static void test_encode_mit(void)
{
    const rs_limits_t *lim = rs_limits_for_model("rs-00");
    rs_can_frame_t f;
    rs_parsed_frame_t p;
    uint16_t pos_u, vel_u, trq_u, kp_u, kd_u;

    printf("[test] encode_mit: id 布局 + 大端 payload\n");
    CHECK(lim != NULL);
    rs_encode_mit(&f, 11, lim, 1.0, -2.0, 30.0, 1.5, 0.5);

    pos_u = rs_scale_signed_u16(1.0, lim->position);
    vel_u = rs_scale_signed_u16(-2.0, lim->velocity);
    trq_u = rs_scale_signed_u16(0.5, lim->torque);
    kp_u  = rs_scale_unsigned_u16(30.0, lim->kp_max);
    kd_u  = rs_scale_unsigned_u16(1.5, lim->kd_max);

    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(p.comm_type == RS_COMM_OP_CTRL);
    CHECK(p.device_id == 11);
    CHECK(p.extra_data == trq_u);           /* 力矩藏在 ID */
    CHECK(f.dlc == 8);
    CHECK(f.data[0] == (uint8_t)(pos_u >> 8));
    CHECK(f.data[1] == (uint8_t)(pos_u & 0xFF));
    CHECK(f.data[2] == (uint8_t)(vel_u >> 8));
    CHECK(f.data[3] == (uint8_t)(vel_u & 0xFF));
    CHECK(f.data[4] == (uint8_t)(kp_u >> 8));
    CHECK(f.data[5] == (uint8_t)(kp_u & 0xFF));
    CHECK(f.data[6] == (uint8_t)(kd_u >> 8));
    CHECK(f.data[7] == (uint8_t)(kd_u & 0xFF));
}

/* -------------------- 缩放互逆 -------------------- */
static void test_signed_scaling_roundtrip(void)
{
    const rs_limits_t *lim = rs_limits_for_model("rs-00");
    const double samples[] = {-4.0, -1.0, 0.0, 1.0, 4.0};

    printf("[test] signed scaling round-trips within 1 LSB\n");
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); i++) {
        uint16_t u = rs_scale_signed_u16(samples[i], lim->position);
        CHECK_NEAR(rs_unscale_signed_u16(u, lim->position), samples[i], 0.001);
    }
}

/* -------------------- 型号表 / kt / √2 -------------------- */
static void test_torque_constant_table(void)
{
    static const struct { const char *m; double kt; } rows[] = {
        {"rs-00", 1.48}, {"rs-01", 1.22}, {"rs-02", 1.22}, {"rs-03", 2.36},
        {"rs-04", 2.10}, {"rs-05", 0.94}, {"rs-06", 1.10},
    };

    printf("[test] torque constant per model\n");
    for (unsigned i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        CHECK(rs_torque_constant_for_model(rows[i].m) == rows[i].kt);
    }
    CHECK(rs_torque_constant_for_model("rs-99") == 0.0);
    CHECK(rs_torque_constant_for_model("") == 0.0);
    CHECK(rs_torque_constant_for_model(NULL) == 0.0);
}

static void test_csp_limit_current_sqrt2(void)
{
    const double kSqrt2 = 1.4142135623730951;
    double written;

    printf("[test] csp_limit_current converts Arms -> Apk (x sqrt2)\n");
    CHECK_NEAR(rs_csp_limit_current_from_torque(5.0, 1.48), 4.778, 0.001);
    CHECK_NEAR(rs_csp_limit_current_from_torque(7.0, 1.22), 8.114, 0.001);
    CHECK_NEAR(rs_csp_limit_current_from_torque(1.8, 0.94), 2.708, 0.001);

    written = rs_csp_limit_current_from_torque(5.0, 1.48);
    CHECK_NEAR(written * 1.48 / kSqrt2, 5.0, 1e-9);
    CHECK_NEAR(5.0 / kSqrt2, 3.5355, 1e-3);      /* 真机实测 3.54 */

    CHECK(rs_csp_limit_current_from_torque(0.0, 1.48) == 0.0);
    CHECK(rs_csp_limit_current_from_torque(5.0, 0.0) == 0.0);
    CHECK(rs_csp_limit_current_from_torque(-1.0, 1.48) == 0.0);
}

static void test_torque_spec_table(void)
{
    static const struct { const char *m; double rated, peak; } rows[] = {
        {"rs-00", 5.0, 14.0},  {"rs-01", 7.0, 17.0}, {"rs-02", 7.0, 17.0},
        {"rs-03", 21.0, 60.0}, {"rs-04", 35.0, 120.0},
        {"rs-05", 1.8, 5.5},   {"rs-06", 11.0, 36.0},
    };

    printf("[test] torque spec table matches spec sheet\n");
    for (unsigned i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
        const rs_torque_spec_t *spec = rs_torque_spec_for_model(rows[i].m);
        CHECK(spec != NULL);
        if (spec != NULL) {
            CHECK(spec->rated == rows[i].rated);
            CHECK(spec->peak == rows[i].peak);
        }
    }
    CHECK(rs_torque_spec_for_model("rs-99") == NULL);
}

static void test_torque_peak_agrees_with_limits(void)
{
    static const char *models[] = {"rs-00", "rs-01", "rs-02", "rs-03",
                                   "rs-04", "rs-05", "rs-06"};

    printf("[test] torque_spec.peak == limits.torque, rated < peak\n");
    for (unsigned i = 0; i < sizeof(models) / sizeof(models[0]); i++) {
        const rs_torque_spec_t *spec = rs_torque_spec_for_model(models[i]);
        const rs_limits_t *lim = rs_limits_for_model(models[i]);
        CHECK(spec != NULL && lim != NULL);
        if (spec != NULL && lim != NULL) {
            CHECK(spec->peak == lim->torque);
            CHECK(spec->rated < spec->peak);
        }
    }
}

/* -------------------- 类型 22 保存 / zero_sta -------------------- */
static void test_save_params_layout(void)
{
    rs_can_frame_t f;
    rs_parsed_frame_t p;

    printf("[test] save_params frame layout 01..08\n");
    rs_encode_save_params(&f, 11, 0xFD);
    CHECK(f.dlc == 8);
    for (int i = 0; i < 8; i++) {
        CHECK(f.data[i] == (uint8_t)(i + 1));
    }
    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(p.comm_type == RS_COMM_SAVE_PARAMS);
    CHECK(p.device_id == 11);
    CHECK(p.extra_data == 0x00FD);
}

static void test_read_zero_sta_request(void)
{
    rs_can_frame_t f;
    rs_parsed_frame_t p;

    printf("[test] read zero_sta request index little-endian\n");
    rs_encode_read_param(&f, 13, 0xFD, RS_PARAM_ZERO_STA);
    CHECK(f.dlc == 8);
    CHECK(f.data[0] == 0x29);
    CHECK(f.data[1] == 0x70);
    for (int i = 2; i < 8; i++) {
        CHECK(f.data[i] == 0);
    }
    CHECK(rs_parse_frame(&f, &p) == 1);
    CHECK(p.comm_type == RS_COMM_READ_PARAM);
    CHECK(p.device_id == 13);
}

/* -------------------- 每轴型号量程 -------------------- */
static void test_limits_for_can_id(void)
{
    printf("[test] per-CAN-ID model limits\n");
    CHECK(rs_limits_for_can_id(11)->torque == 17.0);   /* left shoulder pitch rs-02 */
    CHECK(rs_limits_for_can_id(12)->torque == 14.0);   /* left shoulder roll  rs-00 */
    CHECK(rs_limits_for_can_id(15)->torque == 5.5);    /* left wrist yaw     rs-05 */
    CHECK(rs_limits_for_can_id(18)->torque == 5.5);    /* left gripper       rs-05 */
    CHECK(rs_limits_for_can_id(21)->torque == 17.0);   /* right shoulder pitch */
    CHECK(rs_limits_for_can_id(24)->torque == 14.0);   /* right elbow        rs-00 */
    CHECK(rs_limits_for_can_id(28)->torque == 5.5);    /* right gripper      rs-05 */
    CHECK(rs_limits_for_can_id(99)->torque == 17.0);   /* 未知 ID 兜底 rs-02 */
}

/* -------------------- 故障位描述 -------------------- */
static void test_describe_fault_bits(void)
{
    char buf[64];

    printf("[test] describe_fault_bits\n");
    CHECK(strcmp(rs_describe_fault_bits(0, buf, sizeof(buf)), "") == 0);
    rs_describe_fault_bits((1U << 2) | (1U << 4), buf, sizeof(buf));
    CHECK(strcmp(buf, "过温|堵转过载") == 0);
    rs_describe_fault_bits((1U << 6), buf, sizeof(buf));
    CHECK(strcmp(buf, "未知位") == 0);
}

int main(void)
{
    printf("=== rs_protocol host tests ===\n");

    test_write_param_u8_byte4();
    test_write_param_float_loc_ref();
    test_read_param_request();
    test_decode_read_param();
    test_decode_read_param_rejects();
    test_read_param_scalars();
    test_active_report_byte6();
    test_status_like();
    test_decode_status_active_report();
    test_decode_status_rejects();
    test_status_fault_bits();
    test_encode_mit();
    test_signed_scaling_roundtrip();
    test_torque_constant_table();
    test_csp_limit_current_sqrt2();
    test_torque_spec_table();
    test_torque_peak_agrees_with_limits();
    test_limits_for_can_id();
    test_save_params_layout();
    test_read_zero_sta_request();
    test_describe_fault_bits();

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    if (g_fails == 0) {
        printf("ALL PASS\n");
        return 0;
    }
    printf("FAILED\n");
    return 1;
}
