/**
 * @file dmmotor.h
 * @author Refactored by neozng / Weedy
 * @brief  达妙(DM)系列电机驱动 — 一拖四/集中式控制完整版
 * @version 2.0
 * @date 2026-08-05
 *
 * ============================ 重构说明 ============================
 *
 * 与旧版(原始 dmmotor.h)的核心区别:
 *
 *  旧版                                新版
 *  ─────────────────────────────────────────────────────────────
 *  每个电机独立 FreeRTOS 任务          集中式 DMMotorControl(), 放入 MotorControlTask
 *  pid_ref[3] 数组传参                motor_controller.pid_ref 单一标量, 串级流转
 *  手动写 measure.raw_gyro/gyro       other_xxx_feedback_ptr 指针注入
 *  无电机类型分类                      J4310 / J3507 / J4340
 *  DMMotorSetRef(6个参数)             DMMotorSetRef(motor, ref) 与项目通用框架对齐
 *  无自动分组                          MotorSenderGrouping() 自动分组
 *
 * 与项目电机通用框架 (motor_def.h) 的对齐:
 *  - 相同的实例结构布局 (Motor_Controller_s + Motor_Control_Setting_s)
 *  - 相同的 API 命名 (Init / SetRef / Control / Stop 等)
 *  - 相同的集中式控制模式 (遍历所有实例)
 *  - 相同的分组发送机制 (sender_assignment + sender_enable_flag)
 *
 * === 达妙电机硬件型号 (motor_def.h Motor_Type_e) ===
 *
 *   J4310,   // DM-J4310 系列 (e.g. J4310-2EC)
 *   J3507,   // DM-J3507 系列
 *   J4340,   // DM-J4340 系列
 *
 * @copyright Copyright (c) 2022-2026 HNU YueLu EC all rights reserved
 */
#ifndef DMMOTOR_H
#define DMMOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
/* ======================== 电机数量 & 滤波参数 ======================== */

#define DM_MOTOR_CNT            8       // 达妙电机最大实例数

#define DM_SPEED_SMOOTH_COEF    0.85f   // 速度低通滤波系数 (>0.85)
#define DM_CURRENT_SMOOTH_COEF  0.9f    // 电流低通滤波系数 (>0.9)
#define DM_ECD_ANGLE_COEF       0.043945f       // 360/8192, 编码器值 → 角度
#define DM_ECD_TO_RAD           0.000766990394f // 2*PI/8192, 编码器值 → 弧度

#define DM_ECD_RANGE            8191    // 编码器总刻度 (0-8191)
#define DM_HALF_ECD_RANGE       4096    // 半圈刻度

/* MIT 模式物理量范围 */
#define DM_P_MIN  (-12.5f)              // 位置 (rad)
#define DM_P_MAX   12.5f
#define DM_V_MIN  (-60.0f)              // 速度 (rad/s)
#define DM_V_MAX   60.0f
#define DM_T_MIN  (-18.0f)              // 扭矩 (N·m)
#define DM_T_MAX   18.0f

/* ======================== 控制模式枚举 ======================== */

typedef enum {
    DM_MIT_MODE    = 0,   // MIT 模式: 发送位置/速度/力矩, 驱动侧完成闭环
    DM_POSVEL_MODE = 1,   // 位置-速度模式: 发送位置+速度, 驱动侧完成闭环
    DM_VEL_MODE    = 2,   // 速度模式: 发送速度, 驱动侧完成闭环
    DM_DJI_MODE    = 3,   // DJI 兼容模式: MCU 侧完成串级 PID, 发送电流值
} DMControl_Mode_e;

/* ======================== 电机命令枚举 ======================== */

typedef enum {
    DM_CMD_MOTOR_MODE    = 0xfc,  // 使能, 进入运行模式
    DM_CMD_RESET_MODE    = 0xfd,  // 停止电机
    DM_CMD_ZERO_POSITION = 0xfe,  // 将当前位置设为编码器零点
    DM_CMD_CLEAR_ERROR   = 0xfb,  // 清除过热等错误
} DMMotor_Command_e;

/* ======================== 测量数据结构 ======================== */

/**
 * @brief 达妙电机反馈测量值
 *
 * 一拖四协议反馈字段:
 *   ecd, last_ecd, angle_single_round, speed_aps, real_current,
 *   temperature, total_round, total_angle
 *
 * DM 电机额外字段:
 *   offset_ecd / relative_ecd / relative_angle — 编码器零点偏移后的相对角度
 *   gyro_angle / gyro_rate / gyro_accel — 陀螺仪反馈 (云台专用)
 *   relative_angle_gyro — 陀螺仪相对角度 (rad)
 */
typedef struct {
    /* --- 编码器反馈 (一拖四协议格式) --- */
    uint16_t ecd;                // 当前编码器值 0-8191
    uint16_t last_ecd;           // 上一次编码器值
    float    angle_single_round;  // 单圈角度 (度)
    float    speed_aps;           // 角速度 (度/秒), 低通滤波后
    float    real_current;        // 实际电流 (A), 低通滤波后
    uint8_t  temperature;         // 温度 (℃)

    /* --- 多圈角度 --- */
    int32_t  total_round;         // 累计圈数
    float    total_angle;         // 总角度 = total_round * 360 + angle_single_round

    /* --- 相对角度 (编码器零点偏移后) --- */
    uint16_t offset_ecd;          // 编码器零点偏移值
    int32_t  relative_ecd;        // 相对编码器值
    float    relative_angle;      // 相对角度 (rad)

    /* --- 陀螺仪反馈 (云台专用, 由 GimbalTask 通过指针注入) --- */
    float    gyro_angle;          // 陀螺仪绝对角度 (度) — IMU Yaw/Pitch
    float    gyro_rate;           // 陀螺仪角速度 (度/秒) — IMU Gyro[]
    float    gyro_accel;          // 加速度 (m/s²) — IMU Accel[]
    float    gyro_offset_angle;   // 陀螺仪零点偏移 (度)
    float    relative_angle_gyro; // 陀螺仪相对角度 (rad)

    uint8_t  init_flag;           // 初始角度记录标志
    float    init_angle;          // 初始角度
} DM_Motor_Measure_s;

/* ======================== MIT 模式发送报文 ======================== */

typedef struct {
    uint16_t position_des;  // 位置目标 (uint16, 映射自 DM_P_MIN~DM_P_MAX)
    uint16_t velocity_des;  // 速度目标 (uint12, 映射自 DM_V_MIN~DM_V_MAX)
    uint16_t torque_des;    // 力矩目标 (uint12, 映射自 DM_T_MIN~DM_T_MAX)
    uint16_t Kp;            // 位置环刚度
    uint16_t Kd;            // 速度环阻尼
} DMMotor_Send_MIT_s;

/* ======================== 电机实例结构体 ======================== */

/**
 * @brief 达妙电机实例
 *
 * 通用结构布局: measure → motor_settings → motor_controller → CAN/Daemon
 *
 * DM 特有字段:
 *   control_mode — 区分 MIT/POSVEL/VEL/DJI_MODE
 *   gyro_angle_PID — 陀螺仪角度环 PID (云台专用)
 *   gyro_feedback_ptr — 陀螺仪角度反馈指针 (指向 IMU 角度)
 *   shoot_flag — 发射前馈标志
 */
typedef struct {
    /* --- 电机基本信息 --- */
    Motor_Type_e        motor_type;          // J4310 / J3507 / J4340
    DMControl_Mode_e    control_mode;        // MIT / POSVEL / VEL / DJI_MODE

    /* --- 测量值 --- */
    DM_Motor_Measure_s  measure;

    /* --- 控制配置 (复用 motor_def.h 通用结构) --- */
    Motor_Control_Setting_s motor_settings;  // 闭环类型, 反转, 反馈来源
    Motor_Controller_s      motor_controller;// 三环 PID + 前馈指针 + pid_ref

    /* --- 陀螺仪角度环 PID (云台专用, 与编码器 angle_PID 独立) --- */
    PIDInstance          gyro_angle_PID;     // 陀螺仪角度环 PID
    float               *gyro_feedback_ptr;  // 陀螺仪角度反馈指针 (IMU Yaw/Pitch)

    /* --- CAN 通信 --- */
    CANInstance         *motor_can_instance; // 电机 CAN 实例
    uint8_t              sender_group;       // 发送分组号 (0-3)
    uint8_t              message_num;        // 组内编号 (0-3)

    /* --- 状态标志 --- */
    Motor_Working_Type_e stop_flag;          // 启停标志
    uint8_t              shoot_flag;         // 发射前馈标志

    /* --- 守护/离线监控 --- */
    DaemonInstance      *motor_daemon;       // 离线监控实例
    uint32_t             feed_cnt;           // 反馈计数器
    float                dt;                 // 控制周期 (s)
} DMMotorInstance;

/* ======================== 外部接口 ======================== */

/**
 * @brief 初始化一个达妙电机实例
 *
 * @param config        电机初始化配置 (复用 Motor_Init_Config_s)
 * @param control_mode  控制模式 (MIT / POSVEL / VEL / DJI_MODE)
 * @return DMMotorInstance*  电机实例指针, 应用层需保存
 *
 * @note  与 DJIMotorInit() 接口对齐, 多一个 control_mode 参数
 * @attention tx_id 偏移规则:
 *   MIT_MODE:    tx_id 不变
 *   POSVEL_MODE: tx_id += 0x100
 *   VEL_MODE:    tx_id += 0x200
 *   DJI_MODE:    tx_id += 0x3FE (此时启用 MotorSenderGrouping 分组)
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config, DMControl_Mode_e control_mode);

/**
 * @brief 设定电机目标值 (与 DJIMotorSetRef 对齐)
 *
 * @param motor  电机实例指针
 * @param ref    目标值:
 *               DJI_MODE:    根据 outer_loop_type 传入角度(度)/速度(度每秒)/电流(A)
 *               MIT_MODE:    扭矩目标 (N·m)
 *               POSVEL_MODE: 位置目标 (rad)
 *               VEL_MODE:    速度目标 (rad/s)
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref);

/**
 * @brief 切换反馈数据来源 (与 DJIMotorChangeFeed 对齐)
 *
 * @param motor  电机实例指针
 * @param loop   要切换的闭环 (ANGLE_LOOP / SPEED_LOOP)
 * @param source 反馈来源 (MOTOR_FEED / OTHER_FEED)
 *
 * @note  切换到 OTHER_FEED 前, 确保 other_xxx_feedback_ptr 已指向有效数据
 */
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e source);

/**
 * @brief 修改电机的外层闭环目标 (与 DJIMotorOuterLoop 对齐)
 */
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e outer_loop);

/**
 * @brief 集中式电机控制函数 (与 DJIMotorControl 对齐)
 *
 * 遍历所有已注册 DM 电机, 根据 control_mode 执行对应控制逻辑:
 *  - MIT_MODE:    打包发送力矩目标
 *  - POSVEL_MODE: 打包发送位置+速度
 *  - VEL_MODE:    打包发送速度
 *  - DJI_MODE:    串级 PID 计算, 分组发送电流值
 *
 * @note  放入 MotorControlTask() 中, 以 1kHz 调用:
 *        void MotorControlTask() {
 *            DJIMotorControl();
 *            DMMotorControl();  // ← 新增这一行
 *        }
 */
void DMMotorControl(void);

/**
 * @brief DM 电机控制初始化
 *
 * 打印注册信息, 校验电机数量.
 * 不再为每个电机创建 FreeRTOS 任务 (与旧版 DMMotorControlInit 完全不同).
 *
 * @note  在所有 DMMotorInit() 调用之后, MotorControlTask 启动之前调用一次
 */
void DMMotorControlInit(void);

/* --- 启停控制 --- */
void DMMotorEnable(DMMotorInstance *motor);
void DMMotorStop(DMMotorInstance *motor);

/* --- 辅助功能 --- */
void DMMotorCaliEncoder(DMMotorInstance *motor);                       // 编码器零点校准
void DMMotorSetCommand(DMMotorInstance *motor, DMMotor_Command_e cmd); // 发送控制命令
void DMMotorShootFlag(DMMotorInstance *motor, uint8_t shoot_flag);     // 发射前馈标志
void DMMotorControl(void);
#endif // !DMMOTOR_H