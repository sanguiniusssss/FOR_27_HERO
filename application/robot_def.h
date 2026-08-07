/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"
#include "self_controller.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
//#define CHASSIS_BOARD // 底盘板
//#define GIMBAL_BOARD  // 云台板

#define USE_DT7     // 使用DT7遥控器进行控制
//#define USE_VT13    // 使用VT13(图传接收端)遥控器进行控制
//#define USE_FS      // 使用富斯IBUS协议进行控制 // 注意添加反相器！！！！！

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 机械臂参数
#define CALI_STEP_TIME 100                                              // 校准时间
#define ACTION_STEP_TIME 50                                             // 动作完成判断时间
#define yaw1_MAX 156.7f                                                 // yaw1最大角度
#define yaw1_MIN -128.8f                                                // yaw1轴最小角度

#define yaw2_MAX 26.1f                                                   // yaw2最大角度
#define yaw2_MIN -232.3f                                                // yaw2轴最小角度

#define yaw3_MAX 140.3f                                                 // yaw3最大角度(待修改)
#define yaw3_MIN -139.3f                                                // yaw3轴最小角度

#define PITCH_DIFFER_MAX 82.0f                                          // pitch_differ轴最大角度
#define PITCH_DIFFER_MIN -95.0f                                         // pitch_differ轴最小角度

#define GEAR_RATION_YAW1 (16.0f / 64.0f)                                 // yaw1轴齿轮比
#define GEAR_RATION_YAW2 (16.0f / 36.0f)                                 // yaw2轴齿轮比
#define GEAR_RATION_YAW3 (16.0f / 24.0f)                                 // yaw3轴齿轮比


#define GEAR_RATION_DIFFER (16.0f / 24.0f)                              // 差速器行星齿轮:太阳齿轮
#define REDUCTION_RATIO_YAW1 19.0f                                      // yaw1轴电机减速比
#define REDUCTION_RATIO_YAW2 19.0f                                      // yaw2轴电机减速比
#define REDUCTION_RATIO_YAW3 36.0f                                      // yaw3轴电机减速比

#define REDUCTION_RATIO_DIFFER 36.0f                                    // 差速器电机减速比

#define ROTOR_2_SHAFT_YAW1 -(GEAR_RATION_YAW1 / REDUCTION_RATIO_YAW1)        // 电机转子角度转换到yaw1轴角度 // yaw1齿轮传动反向
#define ROTOR_2_SHAFT_YAW2 -(GEAR_RATION_YAW2 / REDUCTION_RATIO_YAW2)        // 电机转子角度转换到yaw2轴角度 // yaw2齿轮传动反向
#define ROTOR_2_SHAFT_YAW3 -(GEAR_RATION_YAW3 / REDUCTION_RATIO_YAW3)        // 电机转子角度转换到yaw3轴角度 // yaw3齿轮传动反向

#define ROTOR_2_SHAFT_ROLL_DIFFER (1.0f / REDUCTION_RATIO_DIFFER)       // 电机转子角度转换到差速器roll轴角度
#define ROTOR_2_SHAFT_PITCH_DIFFER (1.0f / REDUCTION_RATIO_DIFFER)      // 电机转子角度转换到差速器pitch轴角度

#define SHAFT_2_ROTOR_YAW1 -(REDUCTION_RATIO_YAW1 / GEAR_RATION_YAW1)       // yaw1轴角度转换到电机转子角度 // yaw1齿轮传动反向
#define SHAFT_2_ROTOR_YAW2 -(REDUCTION_RATIO_YAW2 / GEAR_RATION_YAW2)       // yaw2轴角度转换到电机转子角度 // yaw2齿轮传动反向
#define SHAFT_2_ROTOR_YAW3 -(REDUCTION_RATIO_YAW3 / GEAR_RATION_YAW3)       // yaw3轴角度转换到电机转子角度 // yaw3齿轮传动反向

#define SHAFT_2_ROTOR_ROLL_DIFFER REDUCTION_RATIO_DIFFER                // 差速器roll轴角度转换到电机转子角度
#define SHAFT_2_ROTOR_PITCH_DIFFER REDUCTION_RATIO_DIFFER               // 差速器pitch轴角度转换到电机转子角度

#define YAW1_VERTICAL_ANGLE -81                                         // 大yaw轴0点与底盘左右边中垂线的夹角（逆时针为正）
// 滑移参数
#define REDUCTION_RATIO_LIFT 19.0f                                      // 抬升电机减速比

#define LIFT_GEAR_R 31.275f                                             // 抬升齿轮半径

#define LIFT_MAX_DIST 701.4f                                            // 抬升最大距离

#define LIFT_DIST_2_ANGLE (360.0f * REDUCTION_RATIO_LIFT / (2.0f * PI * LIFT_GEAR_R))      // 抬升距离转电机角度(可能待修改) // REDUCTION_RATIO_LIFT 原27.0

#define SAFE_DIST 10.0f                                               // 距抬升顶端机械限位安全距离
#define LIFT_MAX_SAFE_DIST (LIFT_MAX_DIST - SAFE_DIST)                // 抬升最大安全距离(待修改)
#define LIFT_SAFE_HEIGHT 248.52f                                      // 机械臂与底盘无干涉时的安全高度

// **************************************************************************************************************************************** 以下无用
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 175           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE 70           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define YAW_MAX_ANGLE 270           // 云台水平方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define YAW_MIN_ANGLE 10           // 云台水平方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
#define SHOOT_SPEED_MAX 30800         // 连续发射最大射频,单位为发/秒,根据机械设计和电机性能设定
#define SHOOT_SPEED_MIN 23000          // 连续发射最小射频,单位为发/秒,根据机械设计和电机性能设定
// **************************************************************************************************************************************** 以上无用
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 360                                                 // 纵向轴距(前进后退方向)
#define WHEEL_TRACK 400                                                   // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0                                          // 云台旋转中心距底盘几何中心的距离,左右方向,向右为正方向，云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0                                          // 云台旋转中心距底盘几何中心的距离,前后方向,向左为正方向，云台位于正中心时默认设为0
#define RADIUS_WHEEL 77                                                   // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f                                       // 底盘电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define N2V ((2 * PI * RADIUS_WHEEL) / (REDUCTION_RATIO_WHEEL * 60000.0)) // 电机转子转速转轮子转速
#define V2N ((REDUCTION_RATIO_WHEEL * 60000.0) / (2 * PI * RADIUS_WHEEL)) // 轮子转速转电机转子转速
#define SMOOTH_COEF_CHASSIS 0.008f                                        // 底盘电机使用的低通滤波器系数
#define MIN_ACCEL (6 * 3.0f * 0.005f * V2N)
#define MAX_ACCEL (6 * 3.0f * 0.005f * V2N)

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 电磁阀状态
#define VALVE_ALL_CLOSE     ((uint8_t)0b0000)  // 关闭全部阀门

#define VALVE_ARM1          ((uint8_t)0b1000)       // 选中机械臂第一路阀门
#define VALVE_ARM2          ((uint8_t)0b0100)       // 选中机械臂第二路阀门
#define VALVE_T1            ((uint8_t)0b0010)       // 选中矿仓第一路阀门
#define VALVE_T2            ((uint8_t)0b0001)       // 选中矿仓第二路阀门

#define VALVE_ARM_ALL_OPEN  ((uint8_t)0b1100)       // 选中机械臂所有阀门
#define VALVE_T_ALL_OPEN    ((uint8_t)0b0011)       // 选中矿仓所有阀门

#define VALVE_ALL_OPEN      ((uint8_t)0b1111)       // 选中所有阀门

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{

    CHASSIS_ZERO_FORCE = 0, // 电流零输入
    CHASSIS_NORMAL,         // 正常
    CHASSIS_NO_MOVE,        // 不移动
    CHASSIS_ROTATE, // *
    CHASSIS_NO_FOLLOW,// *
    CHASSIS_FOLLOW_GIMBAL_YAW,// *
    CHASSIS_MINING,         // 取矿行进模式
    CHASSIS_CHARGE,         // // 兑矿行进模式 *
} chassis_mode_e;

// 机械臂模式设置
typedef enum
{
    UPPER_ZERO_FORCE = 0,          // 电流零输入
    UPPER_NO_MOVE,                 // 锁定上层机构
    UPPER_CALI,                    // 校准模式
    UPPER_SINGLE_MOTOR,            // 单电机控制模式
    UPPER_SLIVER_MINING,           // 开采银矿，取地矿模式
    UPPER_TWO_SLIVER_MINING,       // 一键开采两个银矿模式
    UPPER_FETCH_ORE_1,             // 取出矿仓矿石模式1
    UPPER_FETCH_ORE_2,             // 取出矿仓矿石模式2
    UPPER_GLOD_MINING,             // 开采金矿模式
    UPPER_STORAGE_ORE_1,           // 存放进矿仓1模式
    UPPER_STORAGE_ORE_2,           // 存放进矿仓2模式
    UPPER_EXCHANGE,                // 自定义控制器兑换模式
} upper_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_NOMOVE = 0,              // 云台锁定模式 // *
    GIMBAL_FREE_MODE,               // 云台自由运动模式
    GIMBAL_FIX_ANGLE_MODE,          // 云台核心控制模式，保持云台yaw轴与地面成固定角度，请在校准模式后再开启
    GIMBAL_SLIVER_MINGING_MODE,     // 云台一位单银模式
    GIMBAL_TWO_SLIVER_MINGING_MODE1,// 云台一位双银模式1
    GIMBAL_TWO_SLIVER_MINGING_MODE2,// 云台一位双银模式2
    GIMBAL_STORAGE_ORE_MODE1,       // 云台存矿仓矿石模式1
    GIMBAL_STORAGE_ORE_MODE2,       // 云台存矿仓矿石模式2
    GIMBAL_FETCH_ORE_MODE1,         // 云台取矿仓矿石模式1
    GIMBAL_FETCH_ORE_MODE2,         // 云台取矿仓矿石模式2
    GIMBAL_GOLD_MINING_MODE,        // 云台取金矿模式
    GIMBAL_RESET,                   // 云台复位
    GIMBAL_ZERO_FORCE,              // 废弃
    GIMBAL_GYRO_MODE,               // 云台陀螺仪控制模式
} gimbal_mode_e;


// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
    float chassis_power_now;
    float chassis_power_last;
} Chassis_Power_Data_s;

/* -------------------------机器人控制数据类型定义-------------------------*/
typedef struct
{
    float yaw1;
    float yaw2;
    float yaw3;
    float pitch_differ;
    float roll_differ;
    float lift_dist;
} Upper_Joint_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot/upper订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
typedef struct
{
    // 控制部分
    float vx; // 前进方向速度
    float vy; // 横移方向速度
    float wz; // 旋转速度

    chassis_mode_e chassis_mode;
    uint8_t pump_mode;
    uint8_t Super_flag;
    float relative_angle;

    // UI部分
    upper_mode_e upper_mode;
    gimbal_mode_e gimbal_mode;
    uint8_t UI_Init_Flag;
    uint8_t Ore_Storage_Flag1; // 矿仓1存放标志位
    uint8_t Ore_Storage_Flag2; // 矿仓2存放标志位
} Chassis_Ctrl_Cmd_s;
typedef struct
{
 uint8_t reserve[4];
} lsy;
typedef struct
{
    Upper_Joint_Data_s joint_data; // 关节数据
    Self_Cntlr_s ctrl_data;       // 自定义控制器数据
    uint8_t cfm_flag : 4;
    uint8_t stop_flag : 1;
    uint16_t getsliver_flag;
    upper_mode_e upper_mode;
} chassis_power_buffer_pid_s;

//cmd发布的数据由upper订阅
typedef struct
{
    Upper_Joint_Data_s joint_data; // 关节数据
    Self_Cntlr_s ctrl_data;       // 自定义控制器数据

    uint8_t cfm_flag : 4;
    uint8_t stop_flag : 1;
    uint16_t getsliver_flag;
    upper_mode_e upper_mode;
} Upper_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;
    float yaw_ecd;              
    float pitch_ecd;           
    float yaw_free_angle;       // 逆时针为正
    float pitch_free_angle;     // 逆时针为正
    float yaw_fixed_angle;      // 逆时针为正
    float pitch_add_angle;
    float yaw_add_angle;
    float yaw1_angle;           // 逆时针为正
    uint8_t shoot_flag;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    uint8_t shoot_single_flag;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    float shoot_num;
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis/upper发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

//    Self_Cntlr_s ctrl_data; // 自定义控制器数据

} Chassis_Upload_Data_s;
typedef struct
{
uint8_t super_cap_v[4];

}SuperCap_Power_s;




//机械臂反馈数据
typedef struct
{
    Upper_Joint_Data_s joint_data; // 上层机构关节数据,单位:度
    uint8_t action_step;
} Upper_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;

    float yaw_free_angle_upload;
    float yaw_fixed_angle_upload;
    float yaw_relative_angle;
} Gimbal_Upload_Data_s;// 云台反馈数据结构体
typedef struct
{
   int a; 
   uint8_t shoot_state; // 发射状态
   uint8_t shoot_flag; // 发射标志位
    // ...
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H