#include "gimbal.h"
#include "robot_def.h"
#include "dm_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"


static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static DMMotorInstance *motor_yaw, *motor_pitch;
static attitude_t *gimbal_IMU_data; // 云台IMU数据
 uint8_t mode_change_flag = 0;//模式转换标志位


void GimbalInit()
{
    gimbal_IMU_data = INS_Init();
    Motor_Init_Config_s gimbal_Yaw_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 90,
                .Ki = 1.0f,
                .Kd = 5.0f,
                .IntegralLimit = 300,
                .CoefA = 2.0f,
                .CoefB = 0.05f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .MaxOut = 15000,
                .Derivative_LPF_RC = 0.005f,
            },
            .speed_PID = {
                .Kp = 15, // 4.5
                .Ki = 0, // 0
                .Kd = 0.01, // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 2000,
            },

            .gyro_PID = {
                .Kp = 15, // 0.4
                .Ki = 0, // 0
                .Kd = 0.01,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 2000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP,
        },
        .motor_type = J4310,
    };
    Motor_Init_Config_s gimbal_Pitch_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 5.0f, // 4.5
                .Ki = 0, // 0
                .Kd = 0, // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
            .speed_PID = {
                .Kp = 10,
                .Ki = 0,  // 0
                .Kd = 0.08,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 50,
            },
            .gyro_PID = {
                .Kp = 18,
                .Ki = 0,
                .Kd = 0.1f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .MaxOut = 50,
                .Derivative_LPF_RC = 0.005f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = J4310,
    };
     //@todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    gimbal_Yaw_config.can_init_config.tx_id = 1;
    gimbal_Yaw_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_yaw = DMMotorInit(&gimbal_Yaw_config, DJI_MODE);
    gimbal_Pitch_config.can_init_config.tx_id = 2;
    gimbal_Pitch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_pitch = DMMotorInit(&gimbal_Pitch_config, DJI_MODE);
    motor_yaw->measure.offset_ecd = 1045;  // Yaw 物理零点偏移(一次标定)
        DMMotorControlInit();
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}
/**
 * @brief 云台初始化模式
 *
 */
static void GimbalReset()
{
    mode_change_flag = 1;

    /* Yaw: 复位到标定零点 */
    DMMotorSetRef(motor_yaw, 0);

    /* Pitch: 复位到当前陀螺仪角度 (SPEED_LOOP 自动初始化 offset_angle) */
    DMMotorSetRef(motor_pitch, 0);
}

/**
 * @brief 云台自由运动模式
 *
 */
static void GimbalFreeMode()
{
    static float  yaw_target_rad = 0;
    static float  pitch_target_deg = 0;
    static uint8_t yaw_first_free = 1;
    static uint8_t pitch_first_free = 1;

    gimbal_feedback_data.yaw_relative_angle = motor_yaw->measure.relative_angle;

    if (mode_change_flag == 1)
    {
        yaw_first_free = 1;
        pitch_first_free = 1;
        mode_change_flag = 0;
    }

    /* ---- Yaw: 编码器闭环 ---- */
    if (yaw_first_free) {
        yaw_target_rad = motor_yaw->measure.relative_angle;
        yaw_first_free = 0;
    }
    yaw_target_rad += gimbal_cmd_recv.yaw_add_angle * (PI / 180.0f);
    if (yaw_target_rad > PI)       yaw_target_rad -= 2.0f * PI;
    if (yaw_target_rad < -PI)      yaw_target_rad += 2.0f * PI;
    DMMotorSetRef(motor_yaw, yaw_target_rad);

    /* ---- Pitch: IMU陀螺仪闭环 (SPEED_LOOP 自动维护 relative_angle_gyro) ---- */
    if (pitch_first_free) {
        pitch_target_deg = motor_pitch->measure.relative_angle_gyro * (180.0f / PI);
        pitch_first_free = 0;
    }
    pitch_target_deg += gimbal_cmd_recv.pitch_add_angle;
    if (pitch_target_deg < -15.0f) pitch_target_deg = -15.0f;
    if (pitch_target_deg >  35.0f) pitch_target_deg =  35.0f;
    DMMotorSetRef(motor_pitch, pitch_target_deg * (PI / 180.0f));
}

static void GimbalModeControl()
{
    static gimbal_mode_e last_mode = GIMBAL_NOMOVE;

    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_RESET) {
        GimbalReset();
        motor_yaw->stop_flag = MOTOR_ENALBED;
        motor_pitch->stop_flag = MOTOR_ENALBED;
    }
    else if (gimbal_cmd_recv.gimbal_mode == GIMBAL_FREE_MODE) {
        /* 从其他模式切回 FREE 时重新锁定当前位置, 防止跳回旧目标 */
        if (last_mode != GIMBAL_FREE_MODE)
            mode_change_flag = 1;
        GimbalFreeMode();
        motor_yaw->stop_flag = MOTOR_ENALBED;
        motor_pitch->stop_flag = MOTOR_ENALBED;
    }
    else if (gimbal_cmd_recv.gimbal_mode == GIMBAL_NOMOVE) {
        motor_yaw->stop_flag = MOTOR_STOP;
        motor_pitch->stop_flag = MOTOR_STOP;
    }

    last_mode = gimbal_cmd_recv.gimbal_mode;
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* 设置 Pitch 陀螺仪数据 (dm_motor 内部控制相对角度计算) */
    DMMotorSetGyro(motor_pitch, gimbal_IMU_data->Pitch, gimbal_IMU_data->Gyro[0]);

    DMMotorShootFlag(motor_pitch, gimbal_cmd_recv.shoot_flag);
    DMMotorShootFlag(motor_yaw, 0);

    gimbal_feedback_data.yaw_relative_angle = motor_yaw->measure.relative_angle;
    GimbalModeControl();

    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}