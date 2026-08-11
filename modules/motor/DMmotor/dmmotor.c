/**
 * @file dmmotor.c
 * @author Weedy
 * @brief  达妙系列电机的驱动
 * @version beta
 * @date 2025-05-01
 *
 * @todo 等待增加 MIT 模式的完整驱动(目前只有力控,即T_ff),等待添加一拖四模式的驱动，等待修改电机控制任务的实现方式
 * 
 * @copyright Copyright (c) 2022
 *
 */

#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static CANInstance sender_assignment[4] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x4FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan2, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x4FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

static uint8_t sender_enable_flag[4] = {0};
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}


static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);
    motor->feed_cnt++;
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DM * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];
        if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
    //  measure->offset_ecd = 1871;
    //  measure->relative_ecd = measure->ecd - measure->offset_ecd;
    // if (measure->relative_ecd > HALF_ECD_RANGE)//4096
    // {
    //     measure->relative_ecd -= ECD_RANGE;//8191
    // }
    // else if (measure->relative_ecd < -HALF_ECD_RANGE)
    // {
    //     measure->relative_ecd += ECD_RANGE; 
    // }
    // measure->relative_angle=measure->relative_ecd*MOTOR_ECD_TO_RAD;





// measure->total_angle = 0;
    // int aaaa=0;
    //  measure->last_position = measure->position;
    // tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    // measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    // tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    // measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    // tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    // measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    // measure->T_Mos = (float)rxbuff[6];
    // measure->T_Rotor = (float)rxbuff[7];
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
    config->rx_id = 0x300 + config->tx_id;

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
    motor->control_mode = Motor_Control_Mode;
   
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->gyro_PID, &config->controller_param_init_config.gyro_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
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
        config->can_init_config.tx_id += 0x100;
        break;
    case VEL_MODE:
        config->can_init_config.tx_id += 0x200;
        break;
    case DJI_MODE:  // 等待修改
        config->can_init_config.tx_id += 0x3FE;
        break;
    default:
        while (1)
            LOGERROR("[dm_motor] undefined control mode!");
        break;
    }
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    // DMMotorCaliEncoder(motor);           // 为了使电机的绝对值编码器起作用，不要在初始化时重新校准编码器零点
    // DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

/**
 * @brief 达妙电机设定目标值
 *
 * @param motor 目标电机指针
 * @param ref1 位置目标值
 * @param ref2 速度目标值
 * @param ref3 电流/扭矩目标值,随电机控制模式而切换
 * @param maker_flag 控制模式标志位
 * 
 * @attention 请根据不同电机模式设置对应需要的目标值,不需要的目标值置 0 防止疯车
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref1, float ref2, float ref3, uint8_t maker_flag)
{
    if (motor == NULL) return;
    motor->pid_ref[0] = ref1;
    motor->pid_ref[1] = ref2;
    motor->pid_ref[2] = ref3;
    motor->maker_flag = maker_flag;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}
void DMMotorShootFlag(DMMotorInstance *motor,uint8_t shoot_flag)
{
    motor->shoot_flag_dm = shoot_flag;
}


void DMMotorStop(DMMotorInstance *motor) // 不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
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
    uint8_t motor_flag;
    int16_t set;

    /* 遍历所有 DM 电机 */
    for (size_t i = 0; i < idx; i++) {
        DMMotorInstance *motor = dm_motor_instance[i];
        Motor_Control_Setting_s *setting = &motor->motor_settings;
        DM_Motor_Measure_s *measure = &motor->measure;
        motor_flag = motor->maker_flag;

        float set1 = motor->pid_ref[0];
        float set2 = motor->pid_ref[1];
        float set3 = motor->pid_ref[2];

        switch (motor->control_mode) {

        case MIT_MODE:
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                set3 *= -1;
            {
                DMMotor_Send_MIT_s mit;
                LIMIT_MIN_MAX(set3, DM_T_MIN, DM_T_MAX);
                mit.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
                mit.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
                mit.torque_des   = float_to_uint(set3, DM_T_MIN, DM_T_MAX, 12);
                mit.Kp = 0; mit.Kd = 0;

                if (motor->stop_flag == MOTOR_STOP)
                    mit.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

                CANInstance *can = motor->motor_can_instace;
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
                set1 *= -1;
            {
                DMMotor_Send_PosVel_s posvel;
                LIMIT_MIN_MAX(set1, DM_P_MIN, DM_P_MAX);
                LIMIT_MIN_MAX(set2, DM_V_MIN, DM_V_MAX);
                posvel.p_des.position_des = set1;
                posvel.v_des.velocity_des = set2;
                if (motor->stop_flag == MOTOR_STOP)
                    posvel.v_des.velocity_des = 0;
                memcpy(motor->motor_can_instace->tx_buff, &posvel, 8);
                CANTransmit(motor->motor_can_instace, 1);
            }
            break;

        case VEL_MODE:
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                set2 *= -1;
            {
                DMMotor_Send_Vel_s vel;
                LIMIT_MIN_MAX(set2, DM_V_MIN, DM_V_MAX);
                vel.v_des.velocity_des = set2;
                if (motor->stop_flag == MOTOR_STOP)
                    vel.v_des.velocity_des = 0;
                memcpy(motor->motor_can_instace->tx_buff, &vel, 4);
                CANTransmit(motor->motor_can_instace, 1);
            }
            break;

        case DJI_MODE:
        {
            float pid_measure, pid_ref, pid_out;
            pid_ref = set2;

            if (motor_flag == 1)  // 陀螺仪模式
            {
                pid_measure = measure->relative_angle_gyro;
                float pid_gyro_out = PIDCalculate(&motor->gyro_PID, pid_measure, pid_ref);
                pid_measure = measure->gyro;
                pid_out = PIDCalculate(&motor->speed_PID, pid_measure, pid_gyro_out);
            }
            else  // 编码器模式
            {
                pid_measure = measure->relative_angle;  // rad, ±π
                pid_out = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
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
        uint16_t can_bus = (m->motor_can_instace->can_handle == &hcan1) ? 1 : 2;
        LOGINFO("[dm_motor] [%d] mode=%d can=%d tx_id=0x%lx rx_id=0x%lx",
                i, m->control_mode, can_bus,
                m->motor_can_instace->tx_id, m->motor_can_instace->rx_id);
    }
}