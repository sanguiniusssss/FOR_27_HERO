#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4// 电机数量

#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DM 0.043945f // (360/8192),将编码器值转化为角度制
#define MOTOR_ECD_TO_RAD 0.000766990394f //     2*  PI  /8192
#define ECD_RANGE       8191
#define HALF_ECD_RANGE  4096
#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-60.0f)
#define DM_V_MAX  60.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

typedef enum 
{
    MIT_MODE = 0,       // MIT模式
    POSVEL_MODE,        // 位置速度模式
    VEL_MODE,           // 速度模式
    DJI_MODE,           // 一拖四模式
} DMControl_Mode_e;

typedef struct 
{
    //基础
    uint8_t id;//电机 ID
    uint8_t state;//电机状态（使能/错误等）
    uint8_t init_flag;//初始化标志位
    uint8_t temperature;//电机温度

    //编码器
    uint16_t ecd;//当前编码器原始值
    uint16_t last_ecd;//上一次编码器原始值，用于计算过圈数
    uint16_t offset_ecd;//编码器零位偏移，用于相对角度计算
    int32_t relative_ecd;//相对编码器值（ecd - offset_ecd），带过圈处理
    float angle_single_round;//单圈角度（0°~360°），由 ecd * ECD_ANGLE_COEF_DM 计算
    int32_t total_round;//累计转动圈数（带符号）
    float total_angle;//累计总角度（total_round * 360 + angle_single_round）
    float init_angle;//初始角度记录
    float relative_angle;//相对角度（弧度制），由 relative_ecd * MOTOR_ECD_TO_RAD 计算

    //陀螺仪
    float gyro_angle;//IMU 当前角度（度），由 GimbalTask() 写入
    float relative_angle_gyro;//陀螺仪相对角度（弧度制），gyro_angle - offest_angle 再归一化
    float gyro;//陀螺仪角速度（度/秒），由 GimbalTask() 写入
    float offest_angle;//陀螺仪角度的零位偏移
    float accel;//加速度数据，由 GimbalTask() 写入

    //速度/电流/扭矩
    float velocity;//电机速度（来自 MIT 模式反馈）
    float speed_aps;//滤波后的速度（角度/秒），由 DMMotorDecode() 计算
    float real_current;//滤波后的实际电流
    float torque;//扭矩值（来自 MIT 模式反馈）
    float position;//位置（来自 MIT 模式反馈）
    float last_position;//上一帧位置（MIT 模式）
    float T_Mos;//MOS 管温度
    float T_Rotor;//转子温度
} DM_Motor_Measure_s;// 电机测量数据结构体

typedef struct
{
    uint16_t position_des;// 位置目标值
    uint16_t velocity_des;// 速度目标值
    uint16_t torque_des;// 扭矩目标值
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_MIT_s;// MIT模式发送数据结构体

typedef struct
{
    union
    {
        float position_des;
        uint8_t data[4];
    } p_des;
    union
    {
        float velocity_des;
        uint8_t data[4];
    } v_des;
} DMMotor_Send_PosVel_s;

typedef struct
{
    union
    {
        float velocity_des;
        uint8_t data[4];
    } v_des;
} DMMotor_Send_Vel_s;

typedef struct
{
    union
    {
        float position_des;
        uint8_t data[4];
    } p_des;
    union
    {
        float velocity_des;
        uint8_t data[4];
    } v_des;
    union
    {
        float torque_des;
        uint8_t data[4];
    } t_des;
} DMMotor_Send_DJI_s;
typedef struct 
{
    DMControl_Mode_e control_mode;// 控制模式
    DM_Motor_Measure_s measure;// 电机测量数据结构体
    Motor_Control_Setting_s motor_settings;// 电机控制设置结构体
    PIDInstance gyro_PID;// 角速度环PID
    PIDInstance speed_PID;// 速度环PID
    PIDInstance angle_PID;// 角度环PID
    PIDInstance current_PID;// 电流环PID
    float *other_angle_feedback_ptr;// 其他角度反馈指针
    float *other_speed_feedback_ptr;// 其他速度反馈指针
    float *speed_feedforward_ptr;// 速度前馈指针
    float *gyro_feedforward_ptr;// 角速度前馈指针
    float pid_ref[3];   // 位置;速度;电流/扭矩的目标值
    float raw_gyro;// 原始角速度    
    float relative_gyro_yaw;// 相对角速度(偏航)
    float relative_gyro_pitch;// 相对角速度(俯仰)
    Motor_Working_Type_e stop_flag;// 电机工作状态
    CANInstance *motor_can_instace;// 电机CAN实例指针
    DaemonInstance *motor_daemon;// 电机守护进程实例指针
    uint32_t lost_cnt;
        // 分组发送设置
    uint8_t sender_group;// 发送分组
    uint8_t message_num;// 发送消息数量
    uint8_t maker_flag;// 
     Motor_Controller_s motor_controller;// 电机控制器结构体
     uint8_t extern_flag;// 外部标志,用于区分不同的控制模式
     uint8_t shoot_flag_dm;// 一拖四模式下的射击标志
 
} DMMotorInstance;// 电机实例结构体

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
} DMMotor_Mode_e;// 电机命令枚举

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config, DMControl_Mode_e Motor_Control_Mode);

void DMMotorSetRef(DMMotorInstance *motor, float ref1, float ref2, float ref3, uint8_t maker_flag, uint8_t extern_flag);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);

void DMMotorCaliEncoder(DMMotorInstance *motor);

void DMMotorControlInit();

void DMMotorShootFlag(DMMotorInstance *motor,uint8_t shoot_flag);
#endif // !DMMOTOR