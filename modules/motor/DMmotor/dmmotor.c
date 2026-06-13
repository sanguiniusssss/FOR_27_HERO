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
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[6] = {0};
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
void DMMotorSetRef(DMMotorInstance *motor, float ref1, float ref2, float ref3, uint8_t maker_flag, uint8_t extern_flag)
{
if (motor == NULL)
{
   return; /* code */
}

    motor->pid_ref[0] = ref1;
    motor->pid_ref[1] = ref2;
    motor->pid_ref[2] = ref3;
    motor->maker_flag = maker_flag;
    motor->extern_flag=extern_flag;
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

        int16_t set;
                int16_t set_P=0;
//@Todo: 目前只实现了力控，更多位控PID等请自行添加 // MIT模式
void DMMotorTask(void const *argument)
{
    // float set1, set2, set3;

     float set1, set2, set3;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    uint8_t motor_flag;
    uint8_t extern_flag;
    DM_Motor_Measure_s *measure = &motor->measure;
    while (1)
    {        
        set1 = motor->pid_ref[0];
        set2 = motor->pid_ref[1];
        set3 = motor->pid_ref[2];
        motor_flag=motor->maker_flag;
        extern_flag=motor->extern_flag;
        switch (motor->control_mode)
        {
        case MIT_MODE:
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                set3 *= -1;
            DMMotor_Send_MIT_s motor_send_mailbox_MIT;
            LIMIT_MIN_MAX(set3, DM_T_MIN, DM_T_MAX);
            motor_send_mailbox_MIT.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox_MIT.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox_MIT.torque_des = float_to_uint(set3, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox_MIT.Kp = 0;
            motor_send_mailbox_MIT.Kd = 0;

            if(motor->stop_flag == MOTOR_STOP)
                motor_send_mailbox_MIT.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

            motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox_MIT.position_des >> 8);
            motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox_MIT.position_des);
            motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox_MIT.velocity_des >> 4);
            motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox_MIT.velocity_des & 0xF) << 4) | (motor_send_mailbox_MIT.Kp >> 8));
            motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox_MIT.Kp);
            motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox_MIT.Kd >> 4);
            motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox_MIT.Kd & 0xF) << 4) | (motor_send_mailbox_MIT.torque_des >> 8));
            motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox_MIT.torque_des);

            CANTransmit(motor->motor_can_instace, 1);
            break;
        case POSVEL_MODE:
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                set1 *= -1;
            DMMotor_Send_PosVel_s motor_send_mailbox_PosVel;
            LIMIT_MIN_MAX(set1, DM_P_MIN, DM_P_MAX);
            LIMIT_MIN_MAX(set2, DM_V_MIN, DM_V_MAX);
            motor_send_mailbox_PosVel.p_des.position_des = set1;
            motor_send_mailbox_PosVel.v_des.velocity_des = set2;

            if(motor->stop_flag == MOTOR_STOP)
                motor_send_mailbox_PosVel.v_des.velocity_des = 0;
                
            memcpy(motor->motor_can_instace->tx_buff, &motor_send_mailbox_PosVel, 8);
            CANTransmit(motor->motor_can_instace, 1);
            break;
        case VEL_MODE:
            if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                set2 *= -1;
            DMMotor_Send_Vel_s motor_send_mailbox_Vel;
            LIMIT_MIN_MAX(set2, DM_V_MIN, DM_V_MAX);
            motor_send_mailbox_Vel.v_des.velocity_des = set2;
            if(motor->stop_flag == MOTOR_STOP)
                motor_send_mailbox_Vel.v_des.velocity_des = 0;

            memcpy(motor->motor_can_instace->tx_buff, &motor_send_mailbox_Vel, 4);
            CANTransmit(motor->motor_can_instace, 1);
            break;
            case DJI_MODE:
            float pid_measure, pid_ref ,pid_out;
 // 如果需要使用motor_controller，确保正确初始化
            pid_ref = set2;
  
            float pid_gyro_out=0,pid_gyro_total=0,pid_accel_out=0;
                if(motor_flag==1)
                {

                     if(extern_flag==0)
                     {
                    pid_measure = measure->relative_angle_gyro;
                    pid_gyro_out = PIDCalculate(&motor->gyro_PID, pid_measure, pid_ref);
                    pid_measure = measure->gyro;
                    pid_out = PIDCalculate(&motor->speed_PID, pid_measure, pid_gyro_total+pid_gyro_out);
                    set = (int16_t)pid_out;
                     }
                     if (extern_flag==1)
                     {
                    // pid_measure=measure->accel;
                    // pid_accel_out=PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
                    pid_measure = measure->relative_angle_gyro;
                    pid_gyro_out = PIDCalculate(&motor->gyro_PID, pid_measure, pid_ref);
                    pid_measure = measure->gyro;
                    pid_out = PIDCalculate(&motor->speed_PID, pid_measure, pid_gyro_total+pid_gyro_out);
                        if(motor->shoot_flag_dm==1)
                        {
                                pid_out=pid_out;
                        }

                    set_P = (int16_t)pid_out;
                     }
                    }
                if(motor_flag==0)
                {
                pid_measure = measure->relative_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
                pid_out = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
                     if(extern_flag==0)
                     {
                         set = (int16_t)pid_out;
                     }
                     if (extern_flag==1)
                     {
                         set_P = (int16_t)pid_out;/* code */
                     }
                }// 更新pid_ref进入下一个环
    // 处理停止标志
    if (motor->stop_flag == MOTOR_STOP)  
    {
     set = 0;
     set_P =0;
    }
     LIMIT_MIN_MAX(set, DM_V_MIN, DM_V_MAX);
     LIMIT_MIN_MAX(set_P, DM_V_MIN, DM_V_MAX);
        if (extern_flag==0)
        {
                sender_assignment[3].tx_buff[2 * (motor->motor_can_instace->tx_id-1-0x3FE)+0] = (uint8_t)(set >> 8);  // 低八位
                sender_assignment[3].tx_buff[2 * (motor->motor_can_instace->tx_id-1-0x3FE) + 1] = (uint8_t)(set & 0x00ff); // 高八位
            CANTransmit(&sender_assignment[3], 1); /* code */
        }
                if (extern_flag==1)
        {
                sender_assignment[0].tx_buff[2 * (motor->motor_can_instace->tx_id-1-0x3FE)+0] = (uint8_t)(set_P >> 8);  // 低八位
                sender_assignment[0].tx_buff[2 * (motor->motor_can_instace->tx_id-1-0x3FE) + 1] = (uint8_t)(set_P & 0x00ff); // 高八位
            CANTransmit(&sender_assignment[0], 1); /* code */
        }


             break;
        default:
            while (1)
                LOGERROR("[dm_motor] undefined control mode!");
            break;
        }

        osDelay(1);
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
        osDelay(1);
    }
}

void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}