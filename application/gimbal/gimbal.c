/**
 * @file gimbal.c
 * @brief 云台应用 — 新版 dm_motor v2.0 一拖四(DJI)模式, DT7遥控器控制
 *
 * ============================ 设计说明 ============================
 *
 * 控制方案:
 *   DT7左摇杆 → yaw_add_angle / pitch_add_angle (增量角度)
 *   → 累加到绝对角度目标 → DMMotorSetRef(绝对角度)
 *   → 新版dm_motor内部完成: 编码器角度环→速度环→电流输出
 *
 * 模式:
 *   GIMBAL_FREE_MODE  — DT7摇杆控制云台转动
 *   GIMBAL_RESET      — 目标角度重置为当前编码器位置
 *   GIMBAL_NOMOVE     — 电机停止
 *
 * @copyright Copyright (c) 2022-2026 HNU YueLu EC all rights reserved
 */

#include "gimbal.h"
#include "robot_def.h"
#include "dm_motor.h"
#include "message_center.h"
#include "general_def.h"

/* ======================== 静态变量 ======================== */

static Publisher_t       *gimbal_pub;
static Subscriber_t      *gimbal_sub;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s    gimbal_cmd_recv;
static DMMotorInstance   *motor_yaw, *motor_pitch;

/* ======================== 初始化 ======================== */

void GimbalInit(void)
{
    /* ---- Yaw 配置 (CAN2, tx_id=1, J4310/M2006级达妙电机) ---- */
    Motor_Init_Config_s yaw_cfg = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 1,
            .rx_id      = 0x301,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 50,
                .Ki            = 0,
                .Kd            = 5,
                .MaxOut        = 15000,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit
                               | PID_Derivative_On_Measurement,
            },
            .speed_PID = {
                .Kp            = 15,
                .Ki            = 0,
                .Kd            = 0.01f,
                .MaxOut        = 2000,
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
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = J4310,
    };
    motor_yaw = DMMotorInit(&yaw_cfg, DM_DJI_MODE);

    /* ---- Pitch 配置 (CAN1, tx_id=2, J3507/M3508级达妙电机) ---- */
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
    static float  yaw_ref   = 0.0f;
    static float  pitch_ref = 0.0f;
    static uint8_t inited   = 0;

    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* ---- 首次运行: 用当前编码器位置初始化目标角度 ---- */
    if (!inited) {
        yaw_ref   = motor_yaw->measure.total_angle;
        pitch_ref = motor_pitch->measure.total_angle;
        inited    = 1;
    }

    /* ---- 模式处理 ---- */
    switch (gimbal_cmd_recv.gimbal_mode) {

    case GIMBAL_RESET:
        /* 复位: 目标角度归零到当前编码器位置 */
        yaw_ref   = motor_yaw->measure.total_angle;
        pitch_ref = motor_pitch->measure.total_angle;
        DMMotorEnable(motor_yaw);
        DMMotorEnable(motor_pitch);
        break;

    case GIMBAL_NOMOVE:
        /* 停止 */
        DMMotorStop(motor_yaw);
        DMMotorStop(motor_pitch);
        PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
        return;

    case GIMBAL_FREE_MODE:
    default:
        /* 正常模式: DT7增量角度累加 */
        DMMotorEnable(motor_yaw);
        DMMotorEnable(motor_pitch);
        break;
    }

    /* ---- 累加 DT7 增量角度命令到绝对目标 ---- */
    yaw_ref   += gimbal_cmd_recv.yaw_add_angle;
    pitch_ref += gimbal_cmd_recv.pitch_add_angle;

    /* ---- 设定电机目标角度 (新版dm_motor内部完成串级PID) ---- */
    DMMotorSetRef(motor_yaw,   yaw_ref);
    DMMotorSetRef(motor_pitch, pitch_ref);

    /* ---- 反馈: yaw相对角度供底盘运动学跟随 ---- */
    gimbal_feedback_data.yaw_relative_angle = motor_yaw->measure.relative_angle;
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
