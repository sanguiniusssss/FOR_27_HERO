/**
 * @file gimbal.c
 * @brief 云台应用 — Pitch 轴控制 (Yaw 轴已迁移至 chassis.c)
 *
 * 控制方案:
 *   DT7左摇杆 → pitch_add_angle → 累加到绝对角度目标
 *   → DMMotorSetRef → dm_motor: 角度环→速度环→CAN帧
 *
 * @copyright Copyright (c) 2022-2026 HNU YueLu EC all rights reserved
 */

#include "gimbal.h"
#include "robot_def.h"
#include "dm_motor.h"
#include "message_center.h"
#include "general_def.h"

/* ======================== 静态变量 ======================== */

static Publisher_t          *gimbal_pub;
static Subscriber_t         *gimbal_sub;
static Gimbal_Upload_Data_s  gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s     gimbal_cmd_recv;
static DMMotorInstance      *motor_pitch;

/* ======================== 初始化 ======================== */

void GimbalInit(void)
{
    /* ---- Pitch 配置 (CAN1, tx_id=2, J4310 达妙电机) ---- */
    Motor_Init_Config_s pitch_cfg = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 2,
            .rx_id      = 0x302,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 100,
                .Ki            = 0,
                .Kd            = 10,
                .MaxOut        = 15000,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit
                               | PID_Derivative_On_Measurement,
            },
            .speed_PID = {
                .Kp            = 20,
                .Ki            = 0,
                .Kd            = 0.08f,
                .MaxOut        = 50,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit
                               | PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
        .motor_type = J4310,
    };
    motor_pitch = DMMotorInit(&pitch_cfg, DM_DJI_MODE);

    DMMotorControlInit();

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd",  sizeof(Gimbal_Ctrl_Cmd_s));
}

/* ======================== 控制任务 (200Hz, RobotTask调用) ======================== */

void GimbalTask(void)
{
    static float  pitch_ref = 0.0f;
    static uint8_t inited   = 0;

    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* 首次运行: 用当前编码器位置初始化目标角度 */
    if (!inited) {
        pitch_ref = motor_pitch->measure.total_angle;
        inited    = 1;
    }

    switch (gimbal_cmd_recv.gimbal_mode) {

    case GIMBAL_RESET:
        pitch_ref = motor_pitch->measure.total_angle;
        DMMotorEnable(motor_pitch);
        break;

    case GIMBAL_NOMOVE:
        DMMotorStop(motor_pitch);
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;

    case GIMBAL_FREE_MODE:
    default:
        DMMotorEnable(motor_pitch);
        break;
    }

    pitch_ref += gimbal_cmd_recv.pitch_add_angle;

    DMMotorSetRef(motor_pitch, pitch_ref);

    /* 反馈: Yaw相对角度由 chassis 侧提供, gimbal 不再负责 */
    gimbal_feedback_data.yaw_relative_angle = 0.0f;
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
