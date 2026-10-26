/**
 * @file    sim_can.c
 * @brief   16 台虚拟 RobStride 电机
 */
#include <math.h>
#include <string.h>

#include "sim_can.h"

#define SIM_MOTOR_COUNT     16U
#define SIM_ID_BASE         1U
#define SIM_QUEUE_LEN       64U

typedef struct {
    uint8_t  enabled;
    uint8_t  offline;
    uint8_t  fault_bits;
    double   pos;           /* rad */
    double   vel;
    double   target;
    double   vel_cmd;
    double   torque;
    double   kp;
    double   kd;
    double   temp;
} sim_motor_t;

static sim_motor_t s_motors[SIM_MOTOR_COUNT];

static rs_can_frame_t s_queue[SIM_QUEUE_LEN];
static uint32_t       s_q_head;
static uint32_t       s_q_tail;


static void queue_push(const rs_can_frame_t *f)
{
    uint32_t next = (s_q_head + 1U) % SIM_QUEUE_LEN;

    if (next == s_q_tail) {
        return;                          /* 满了丢新帧，模拟总线拥塞 */
    }
    s_queue[s_q_head] = *f;
    s_q_head = next;
}

static sim_motor_t *motor_by_id(uint8_t id, uint8_t *index)
{
    if (id < SIM_ID_BASE || id >= SIM_ID_BASE + SIM_MOTOR_COUNT) {
        return NULL;
    }
    if (index != NULL) {
        *index = (uint8_t)(id - SIM_ID_BASE);
    }
    return &s_motors[id - SIM_ID_BASE];
}

void Sim_Init(void)
{
    memset(s_motors, 0, sizeof(s_motors));
    s_q_head = 0;
    s_q_tail = 0;

    for (uint8_t i = 0; i < SIM_MOTOR_COUNT; i++) {
        s_motors[i].temp = 25.0 + (double)i * 0.3;
        s_motors[i].pos  = (double)((int)i - 8) * 0.01;   /* 轻微不均匀，像真机 */
        s_motors[i].target = s_motors[i].pos;
        s_motors[i].kp = 30.0;
        s_motors[i].kd = 1.0;
    }
}

static void push_status(uint8_t id, const sim_motor_t *m)
{
    const rs_limits_t *lim = rs_limits_for_can_id(id);
    rs_can_frame_t f;
    uint16_t pos_u = rs_scale_signed_u16(m->pos, lim->position);
    uint16_t vel_u = rs_scale_signed_u16(m->vel, lim->velocity);
    uint16_t trq_u = rs_scale_signed_u16(m->torque, lim->torque);
    uint16_t tmp_u = (uint16_t)(m->temp * 10.0);

    f.bus = 0;
    f.id  = rs_make_id(RS_COMM_OP_STATUS,
                       (uint16_t)(((uint16_t)m->fault_bits << 8) | id),
                       (uint8_t)RS_HOST_ID_DEFAULT);
    f.dlc = 8;
    f.data[0] = (uint8_t)(pos_u >> 8);
    f.data[1] = (uint8_t)(pos_u & 0xFFU);
    f.data[2] = (uint8_t)(vel_u >> 8);
    f.data[3] = (uint8_t)(vel_u & 0xFFU);
    f.data[4] = (uint8_t)(trq_u >> 8);
    f.data[5] = (uint8_t)(trq_u & 0xFFU);
    f.data[6] = (uint8_t)(tmp_u >> 8);
    f.data[7] = (uint8_t)(tmp_u & 0xFFU);
    queue_push(&f);
}

void Sim_Tick(void)
{
    const double dt = 0.01;

    for (uint8_t i = 0; i < SIM_MOTOR_COUNT; i++) {
        sim_motor_t *m = &s_motors[i];

        if (m->offline) {
            continue;
        }
        if (m->enabled) {
            /* 一阶跟随 + 力矩记录（够录制/回放逻辑用） */
            m->vel = (m->target - m->pos) * 8.0;
            if (m->vel > 40.0) { m->vel = 40.0; }
            if (m->vel < -40.0) { m->vel = -40.0; }
            m->pos += m->vel * dt;
            m->torque = m->kp * (m->target - m->pos) + m->kd * m->vel;
        } else {
            m->vel = 0.0;
            m->torque = 0.0;
        }
        m->temp += 0.0005 * fabs(m->torque);
        if (m->temp > 80.0) { m->temp = 80.0; }
    }
}

static void reply_read_param(uint8_t id, uint16_t param_id, const sim_motor_t *m)
{
    rs_can_frame_t f;
    float v = 0.0F;

    if (param_id == RS_PARAM_MECH_POS) {
        v = (float)m->pos;
    } else if (param_id == RS_PARAM_MECH_VEL) {
        v = (float)m->vel;
    } else if (param_id == RS_PARAM_RUN_MODE) {
        f.id  = rs_make_id(RS_COMM_READ_PARAM,
                           (uint16_t)(((uint16_t)RS_HOST_ID_DEFAULT << 8) | id), 0);
        f.id  = rs_make_id(RS_COMM_READ_PARAM, (uint16_t)RS_HOST_ID_DEFAULT, id);
        f.dlc = 8;
        f.data[0] = (uint8_t)(param_id & 0xFFU);
        f.data[1] = (uint8_t)(param_id >> 8);
        f.data[2] = 0; f.data[3] = 0;
        f.data[4] = m->enabled ? 1U : 0U;
        f.data[5] = 0; f.data[6] = 0; f.data[7] = 0;
        queue_push(&f);
        return;
    } else {
        v = 0.0F;
    }

    f.bus = 0;
    f.id  = rs_make_id(RS_COMM_READ_PARAM, (uint16_t)RS_HOST_ID_DEFAULT, id);
    f.dlc = 8;
    f.data[0] = (uint8_t)(param_id & 0xFFU);
    f.data[1] = (uint8_t)(param_id >> 8);
    f.data[2] = 0;
    f.data[3] = 0;
    memcpy(&f.data[4], &v, 4);
    queue_push(&f);
}

int Sim_HandleTx(const rs_can_frame_t *frame)
{
    rs_parsed_frame_t p;
    sim_motor_t      *m;
    uint8_t           idx;

    if (!rs_parse_frame(frame, &p)) {
        return 0;
    }
    m = motor_by_id(p.device_id, &idx);
    if (m == NULL) {
        return 1;
    }
    if (m->offline) {
        return 1;                        /* 掉线：不回任何帧 */
    }

    switch (p.comm_type) {
        case RS_COMM_OP_CTRL: {
            const rs_limits_t *lim = rs_limits_for_can_id(p.device_id);
            uint16_t pos_u = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
            uint16_t vel_u = (uint16_t)((frame->data[2] << 8) | frame->data[3]);
            uint16_t kp_u  = (uint16_t)((frame->data[4] << 8) | frame->data[5]);
            uint16_t kd_u  = (uint16_t)((frame->data[6] << 8) | frame->data[7]);

            m->target  = rs_unscale_signed_u16(pos_u, lim->position);
            m->vel_cmd = rs_unscale_signed_u16(vel_u, lim->velocity);
            m->kp      = (double)kp_u / 65535.0 * lim->kp_max;
            m->kd      = (double)kd_u / 65535.0 * lim->kd_max;
            push_status(p.device_id, m);
            break;
        }
        case RS_COMM_ENABLE:
            m->enabled = 1;
            m->target  = m->pos;
            push_status(p.device_id, m);
            break;

        case RS_COMM_DISABLE:
            m->enabled = 0;
            m->torque  = 0.0;
            m->vel     = 0.0;
            push_status(p.device_id, m);
            break;

        case RS_COMM_SET_ZERO:
            m->pos    = 0.0;
            m->vel    = 0.0;
            m->target = 0.0;
            push_status(p.device_id, m);
            break;

        case RS_COMM_READ_PARAM: {
            uint16_t param_id = (uint16_t)(frame->data[0] | (frame->data[1] << 8));
            reply_read_param(p.device_id, param_id, m);
            break;
        }

        case RS_COMM_SAVE_PARAMS:
        case RS_COMM_ACTIVE_REPORT:
        case RS_COMM_GET_ID:
        case RS_COMM_FAULT_REPORT:
            push_status(p.device_id, m);
            break;

        default:
            break;
    }
    (void)idx;
    return 1;
}

int Sim_RxPop(rs_can_frame_t *frame)
{
    if (s_q_tail == s_q_head) {
        return 0;
    }
    *frame = s_queue[s_q_tail];
    s_q_tail = (s_q_tail + 1U) % SIM_QUEUE_LEN;
    return 1;
}

void Sim_SetOffline(uint8_t id, int offline)
{
    sim_motor_t *m = motor_by_id(id, NULL);
    if (m != NULL) {
        m->offline = (uint8_t)(offline ? 1 : 0);
    }
}

void Sim_SetFault(uint8_t id, uint8_t fault_bits)
{
    sim_motor_t *m = motor_by_id(id, NULL);
    if (m != NULL) {
        m->fault_bits = fault_bits;
    }
}

void Sim_MoveTarget(uint8_t id, double pos_rad)
{
    sim_motor_t *m = motor_by_id(id, NULL);
    if (m != NULL) {
        m->target = pos_rad;
        m->enabled = 1;
    }
}

int Sim_GetPos(uint8_t id, double *pos)
{
    sim_motor_t *m = motor_by_id(id, NULL);
    if (m == NULL || pos == NULL) {
        return 0;
    }
    *pos = m->pos;
    return 1;
}

int Sim_GetEnabled(uint8_t id)
{
    sim_motor_t *m = motor_by_id(id, NULL);
    return (m != NULL) ? m->enabled : 0;
}
