/**
 * @file dm_motor.c
 * @author Refactored
 * @brief  达妙(DM)系列电机驱动 — 一拖四/集中式控制
 * @version 2.0
 * @date   2026-08-11
 *
 * 架构:
 *  - 集中式 DMMotorControl(), 在 MotorControlTask 中以 1kHz 调用
 *  - DJI_MODE: 串级 PID (编码器模式 angle_PID, 陀螺仪模式 gyro_PID→speed_PID)
 *  - MIT / POSVEL / VEL: 直发模式
 *  - MotorSenderGrouping(): 自动 CAN 分组 (0x3FE/0x4FE)
 *
 * @copyright Copyright (c) 2022-2026 HNU YueLu EC all rights reserved
 */

#include "dm_motor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include <math.h>

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static CANInstance sender_assignment[4] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = DM_CTRL_ID_1TO4, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = DM_CTRL_ID_5TO8, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan2, .txconf.StdId = DM_CTRL_ID_1TO4, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = DM_CTRL_ID_5TO8, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

static uint8_t sender_enable_flag[4] = {0};
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instance->tx_buff, DM_CMD_HEADER_PAD, DM_CMD_HEADER_LEN);
    motor->motor_can_instance->tx_buff[DM_CMD_HEADER_LEN] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instance, 1);
}


static void DMMotorDecode(CANInstance *motor_can)
{
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);
    motor->feed_cnt++;
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = DM_ECD_ANGLE_COEF * (float)measure->ecd;
    measure->speed_aps = (1.0f - DM_SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * DM_SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - DM_CURRENT_SMOOTH_COEF) * measure->real_current +
                            DM_CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];
        if (measure->ecd - measure->last_ecd > DM_HALF_ECD_RANGE)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -DM_HALF_ECD_RANGE)
        measure->total_round++;
    measure->total_angle = measure->total_round * DM_ANGLE_360 + measure->angle_single_round;
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor_ptr); // 防止因为电机失能导致无返回值
    DWT_Delay(0.1);
}

/** @brief DJI_MODE 电机分组: 自动分配 sender_group 和 rx_id */
static void MotorSenderGrouping(DMMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1;  // motor_ID 1-8 → 索引 0-7
    if (motor_id >= 8) return;

    /* 反馈 CAN ID = 0x300 + motor_ID (一拖四手册) */
    config->rx_id = DM_FEEDBACK_ID_BASE + config->tx_id;

    /* 分组: motor 1-4 → 0x3FE, motor 5-8 → 0x4FE */
    if (motor_id < 4) {
        motor->message_num = motor_id;
        motor->sender_group = (config->can_handle == &hcan1) ? 0 : 2;
    } else {
        motor->message_num = motor_id - 4;
        motor->sender_group = (config->can_handle == &hcan1) ? 1 : 3;
    }
    sender_enable_flag[motor->sender_group] = 1;
}

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

/**
 * @brief 达妙电机的初始化函数
 *
 * @param config 初始化数据指针
 * @param Motor_Control_Mode 达妙电机控制模式选择
 * 
 * @attention 注意电机初始参数与上位机互相对应!!
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config, DMControl_Mode_e Motor_Control_Mode)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    motor->motor_type    = config->motor_type;
    motor->control_mode = Motor_Control_Mode;

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->gyro_PID, &config->controller_param_init_config.gyro_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = DM_DAEMON_RELOAD_CNT,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    /* DJI_MODE: 在 tx_id 偏移前分组 (需要原始 motor_ID) */
    if (motor->control_mode == DJI_MODE)
        MotorSenderGrouping(motor, &config->can_init_config);

    switch (motor->control_mode)
    {
    case MIT_MODE:
        break;
    case POSVEL_MODE:
        config->can_init_config.tx_id += DM_TX_ID_OFFSET_POSVEL;
        break;
    case VEL_MODE:
        config->can_init_config.tx_id += DM_TX_ID_OFFSET_VEL;
        break;
    case DJI_MODE:  // 等待修改
        config->can_init_config.tx_id += DM_TX_ID_OFFSET_DJI;
        break;
    default:
        while (1)
            LOGERROR("[dm_motor] undefined control mode!");
        break;
    }
    motor->motor_can_instance = CANRegister(&config->can_init_config);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    // DMMotorCaliEncoder(motor);           // 为了使电机的绝对值编码器起作用，不要在初始化时重新校准编码器零点
    // DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

/**
 * @brief 设定电机目标值 (与 DJIMotorSetRef 对齐)
 *
 * @param motor  电机实例
 * @param ref    目标值: DJI_MODE 为编码器角度(rad), MIT=扭矩(N·m), etc.
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    if (motor == NULL) return;
    motor->motor_controller.pid_ref = ref;
}


void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}
void DMMotorShootFlag(DMMotorInstance *motor, uint8_t shoot_flag)
{
    (void)motor;
    (void)shoot_flag;
    // 预留接口, 暂未实现
}


void DMMotorStop(DMMotorInstance *motor) // 不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

/**
 * @brief 设置电机的陀螺仪数据 (IMU角度和角速度)
 *
 * @param motor     电机实例
 * @param angle_deg IMU角度(度)
 * @param rate_dps  陀螺仪角速度(dps)
 */
void DMMotorSetGyro(DMMotorInstance *motor, float angle_deg, float rate_dps)
{
    if (motor == NULL) return;
    motor->raw_gyro = angle_deg;
    motor->measure.gyro_angle = angle_deg;
    motor->measure.gyro = rate_dps;
}

/* ======================== 集中式控制 ======================== */


/**
 * @brief 集中式 DM 电机控制 (1kHz, 在 MotorControlTask 中调用)
 *
 * 替换旧的每电机一个 FreeRTOS 任务架构.
 * 遍历所有电机, DJI_MODE 填分组缓冲区后统一发送.
 */
void DMMotorControl(void)
{
    int16_t set;

    /* 遍历所有 DM 电机 */
    for (size_t i = 0; i < idx; i++) {
        DMMotorInstance *motor = dm_motor_instance[i];
        Motor_Control_Setting_s *setting = &motor->motor_settings;
        DM_Motor_Measure_s *measure = &motor->measure;
        float ref = motor->motor_controller.pid_ref;

        /* 编码器 -> relative_angle (原 ecd_relative) */
        if (motor->control_mode == DJI_MODE) {
            measure->relative_ecd = measure->ecd - measure->offset_ecd;
            while (measure->relative_ecd > DM_HALF_ECD_RANGE)
                measure->relative_ecd -= DM_ECD_RANGE;
            while (measure->relative_ecd < -DM_HALF_ECD_RANGE)
                measure->relative_ecd += DM_ECD_RANGE;
            measure->relative_angle = measure->relative_ecd * DM_ECD_TO_RAD;
        }

        switch (motor->control_mode) {

        case MIT_MODE:
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                ref *= -1;
            {
                DMMotor_Send_MIT_s mit;
                LIMIT_MIN_MAX(ref, DM_T_MIN, DM_T_MAX);
                mit.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
                mit.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
                mit.torque_des   = float_to_uint(ref, DM_T_MIN, DM_T_MAX, 12);
                mit.Kp = 0; mit.Kd = 0;

                if (motor->stop_flag == MOTOR_STOP)
                    mit.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

                CANInstance *can = motor->motor_can_instance;
                can->tx_buff[0] = (uint8_t)(mit.position_des >> 8);
                can->tx_buff[1] = (uint8_t)(mit.position_des);
                can->tx_buff[2] = (uint8_t)(mit.velocity_des >> 4);
                can->tx_buff[3] = (uint8_t)(((mit.velocity_des & 0xF) << 4) | (mit.Kp >> 8));
                can->tx_buff[4] = (uint8_t)(mit.Kp);
                can->tx_buff[5] = (uint8_t)(mit.Kd >> 4);
                can->tx_buff[6] = (uint8_t)(((mit.Kd & 0xF) << 4) | (mit.torque_des >> 8));
                can->tx_buff[7] = (uint8_t)(mit.torque_des);
                CANTransmit(can, 1);
            }
            break;

        case POSVEL_MODE:
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                ref *= -1;
            {
                DMMotor_Send_PosVel_s posvel;
                LIMIT_MIN_MAX(ref, DM_P_MIN, DM_P_MAX);
                float vel_ff = 0;
                if (motor->motor_controller.speed_feedforward_ptr)
                    vel_ff = *motor->motor_controller.speed_feedforward_ptr;
                LIMIT_MIN_MAX(vel_ff, DM_V_MIN, DM_V_MAX);
                posvel.p_des.position_des = ref;
                posvel.v_des.velocity_des = vel_ff;
                if (motor->stop_flag == MOTOR_STOP)
                    posvel.v_des.velocity_des = 0;
                memcpy(motor->motor_can_instance->tx_buff, &posvel, 8);
                CANTransmit(motor->motor_can_instance, 1);
            }
            break;

        case VEL_MODE:
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                ref *= -1;
            {
                DMMotor_Send_Vel_s vel;
                LIMIT_MIN_MAX(ref, DM_V_MIN, DM_V_MAX);
                vel.v_des.velocity_des = ref;
                if (motor->stop_flag == MOTOR_STOP)
                    vel.v_des.velocity_des = 0;
                memcpy(motor->motor_can_instance->tx_buff, &vel, 4);
                CANTransmit(motor->motor_can_instance, 1);
            }
            break;

        case DJI_MODE:
        {
            float pid_measure, pid_out;

            if (setting->outer_loop_type == SPEED_LOOP)  // 陀螺仪模式
            {
                /* 陀螺仪相对角度计算 (原 gyro_relative)
                   自动初始化 offset_angle */
                if (fabsf(measure->offset_angle - motor->raw_gyro) > DM_GYRO_INIT_THRESHOLD)
                    measure->offset_angle = motor->raw_gyro;
                measure->relative_angle_gyro = measure->gyro_angle - measure->offset_angle;
                while (measure->relative_angle_gyro > DM_ANGLE_180)
                    measure->relative_angle_gyro -= DM_ANGLE_360;
                while (measure->relative_angle_gyro < -DM_ANGLE_180)
                    measure->relative_angle_gyro += DM_ANGLE_360;
                while (measure->relative_angle_gyro > DM_ANGLE_180)
                    measure->relative_angle_gyro -= DM_ANGLE_360;
                while (measure->relative_angle_gyro < -DM_ANGLE_180)
                    measure->relative_angle_gyro += DM_ANGLE_360;
                measure->relative_angle_gyro *= ((float)PI / DM_ANGLE_180);  // 转弧度

                pid_measure = measure->relative_angle_gyro;
                float pid_gyro_out = PIDCalculate(&motor->gyro_PID, pid_measure, ref);
                pid_measure = measure->gyro;
                pid_out = PIDCalculate(&motor->speed_PID, pid_measure, pid_gyro_out);
            }
            else  // 编码器模式 (ANGLE_LOOP)
            {
                pid_measure = measure->relative_angle;  // rad, ±π

                /* 解缠: 将 ±π 内折叠的测量值展开为与上次连续的坐标
                   用 roundf 处理任意圈数, 不仅是 ±1 圈 */
                float last_m = motor->angle_PID.Last_Measure;
                float delta = pid_measure - last_m;
                if (fabsf(delta) > PI) {
                    float wraps = roundf(delta / (2.0f * PI));
                    pid_measure -= wraps * 2.0f * PI;
                }

                /* ref 是 ±π 内的角度目标, pid_measure 已解缠到连续空间.
                   将 ref 映射到离 pid_measure 最近的 2π 等效值,
                   保证误差 ≤ π (始终走最短路径回正) */
                float wraps = roundf((pid_measure - ref) / (2.0f * PI));
                float ref_cont = ref + wraps * 2.0f * PI;
                pid_out = PIDCalculate(&motor->angle_PID, pid_measure, ref_cont);
            }

            if (motor->stop_flag == MOTOR_STOP)
                pid_out = 0;

            set = (int16_t)pid_out;
            LIMIT_MIN_MAX(set, DM_V_MIN, DM_V_MAX);

            sender_assignment[motor->sender_group].tx_buff[2 * motor->message_num + 0] = (uint8_t)(set >> 8);
            sender_assignment[motor->sender_group].tx_buff[2 * motor->message_num + 1] = (uint8_t)(set & 0x00ff);
            break;
        }

        default:
            break;
        }
    }

    /* DJI_MODE: 统一发送所有活跃分组 */
    for (size_t i = 0; i < 4; i++) {
        if (sender_enable_flag[i])
            CANTransmit(&sender_assignment[i], 1);
    }

    /* 保活: 每周期仅使能的电机发送 (0xFC 命令帧) */
    for (size_t i = 0; i < idx; i++) {
        DMMotorInstance *motor = dm_motor_instance[i];
        if (motor->control_mode == DJI_MODE && motor->stop_flag == MOTOR_ENALBED)
            DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    }
}

void DMMotorControlInit(void)
{
    if (idx == 0) {
        LOGINFO("[dm_motor] No DM motor registered, skip init.");
        return;
    }
    LOGINFO("[dm_motor] %d motor(s) registered, centralized control ready.", idx);
    for (size_t i = 0; i < idx; i++) {
        DMMotorInstance *m = dm_motor_instance[i];
        uint16_t can_bus = (m->motor_can_instance->can_handle == &hcan1) ? 1 : 2;
        LOGINFO("[dm_motor] [%d] mode=%d can=%d tx_id=0x%lx rx_id=0x%lx",
                i, m->control_mode, can_bus,
                m->motor_can_instance->tx_id, m->motor_can_instance->rx_id);
    }
}