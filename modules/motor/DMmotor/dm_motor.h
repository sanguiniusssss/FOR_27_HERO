#ifndef DM_MOTOR_H
#define DM_MOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define DM_CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define DM_ECD_ANGLE_COEF 0.043945f // (360/8192),将编码器值转化为角度制
#define DM_ECD_TO_RAD 0.000766990394f //     2*  PI  /8192
#define DM_ECD_RANGE       8191
#define DM_HALF_ECD_RANGE  4096
/* ========== 协议常量 ========== */
#define DM_CMD_HEADER_LEN         7
#define DM_CMD_HEADER_PAD         0xFF
#define DM_FEEDBACK_ID_BASE       0x300
#define DM_CTRL_ID_1TO4           0x3FE
#define DM_CTRL_ID_5TO8           0x4FE
#define DM_TX_ID_OFFSET_POSVEL    0x100
#define DM_TX_ID_OFFSET_VEL       0x200
#define DM_TX_ID_OFFSET_DJI       0x3FE

/* ========== 阈值 ========== */
#define DM_DAEMON_RELOAD_CNT      10
#define DM_GYRO_INIT_THRESHOLD    90.0f

/* ========== 角度常量 ========== */
#define DM_ANGLE_360              360.0f
#define DM_ANGLE_180              180.0f
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
    float velocity;
    float position;
    int32_t total_round;
    uint16_t ecd;
    uint16_t last_ecd;
    float angle_single_round;
    float total_angle;
    float init_angle;
    float speed_aps;
    float real_current;
    uint8_t temperature;
    uint8_t init_flag;
    uint16_t offset_ecd;
    int32_t relative_ecd;
    float relative_angle;
    float gyro_angle;
    float relative_angle_gyro;
    float gyro;
    float offset_angle;
} DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_MIT_s;

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
    Motor_Type_e        motor_type;
    DMControl_Mode_e control_mode;
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance gyro_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;

    float raw_gyro;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instance;
    DaemonInstance *motor_daemon;
    uint32_t feed_cnt;       // CAN反馈计数器
        // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;
     Motor_Controller_s motor_controller;
 
} DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
} DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config, DMControl_Mode_e Motor_Control_Mode);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);

void DMMotorCaliEncoder(DMMotorInstance *motor);

void DMMotorControlInit(void);

void DMMotorControl(void);

void DMMotorShootFlag(DMMotorInstance *motor,uint8_t shoot_flag);

void DMMotorSetGyro(DMMotorInstance *motor, float angle_deg, float rate_dps);
#endif // !DM_MOTOR_H