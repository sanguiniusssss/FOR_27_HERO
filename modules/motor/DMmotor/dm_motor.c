/**
 * @file dmmotor.c
 * @author Refactored by neozng / Weedy
 * @brief  达妙(DM)系列电机驱动 — DJI风格完整版
 * @version 2.0
 * @date 2026-08-05
 *
 * ============================ 架构说明 ============================
 *
 * 控制流程:
 *   DMMotorInit() × N  →  DMMotorControlInit()  →  MotorControlTask()
 *                                                       │
 *                                               DMMotorControl()  ← 1kHz
 *                                                       │
 *                                          ┌────────────┼────────────┐
 *                                     MIT_MODE    POSVEL/VEL    DJI_MODE
 *                                     (直发)      (直发)      (串级PID)
 *
 * 与旧版核心区别:
 *  旧版: 每个电机一个 FreeRTOS 任务 (DMMotorTask), 8电机 = 8任务 × 512B = 4KB 栈
 *  新版: 集中式 DMMotorControl(), 0 额外任务开销
 *
 * 与 DJI 电机驱动对齐:
 *  - Motor_Controller_s 统一框架 (pid_ref 串级流转)
 *  - MotorSenderGrouping() 分组 (与 DJI 共用 sender_assignment 逻辑)
 *  - other_xxx_feedback_ptr 注入陀螺仪反馈
 *
 * @copyright Copyright (c) 2022-2026 HNU YueLu EC all rights reserved
 */

#include "dm_motor.h"

#include "general_def.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "daemon.h"
#include "stdlib.h"
#include "string.h"
#include "memory.h"

/* ======================== 静态变量 ======================== */

static uint8_t          idx = 0;                               // 已注册电机计数
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT] = {NULL}; // 实例指针数组

/**
 * @brief DM 电机分组发送表 (DJI_MODE 专用)
 *
 * 与 DJI 的 sender_assignment 逻辑相同, 但 CAN ID 不同:
 *   DJI:  0x1FF (C610/C620 group1) / 0x200 (group2) / 0x2FF (GM6020)
 *   DM:   0x3FE (group1)           / 0x200 (group2) / 0x2FF (group3)
 *
 * can1: [0]:0x3FE, [1]:0x200, [2]:0x2FF
 * can2: [3]:0x3FE, [4]:0x200, [5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

static uint8_t sender_enable_flag[6] = {0}; // 分组发送使能标志

/* ======================== 私有: MIT 协议 float ↔ uint 映射 ======================== */

/**
 * @brief 浮点物理量 → MIT 协议 uint 值
 * @param x     浮点值
 * @param x_min 物理量最小值
 * @param x_max 物理量最大值
 * @param bits  协议位宽
 * @return uint16_t 映射后的无符号整型
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief MIT 协议 uint 值 → 浮点物理量
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ======================== 私有: 电机命令发送 ======================== */

/**
 * @brief 发送达妙电机控制命令 (使能/停止/校零/清除错误)
 *
 * 协议: tx_buff[0..6] = 0xFF, tx_buff[7] = cmd
 */
void DMMotorSetCommand(DMMotorInstance *motor, DMMotor_Command_e cmd)
{
    if (motor == NULL || motor->motor_can_instance == NULL)
        return;

    memset(motor->motor_can_instance->tx_buff, 0xff, 7);
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instance, 1);
}

/* ======================== 私有: CAN 反馈解码 ======================== */

/**
 * @brief 达妙电机 CAN 反馈解码回调
 *
 * 与 DJI 的 DecodeDJIMotor() 对齐:
 *  1. 喂狗 (DaemonReload)
 *  2. 解析编码器 → 单圈角度
 *  3. 解析速度 → 低通滤波 → 度/秒
 *  4. 解析电流 → 低通滤波 → A
 *  5. 温度
 *  6. 多圈角度计算
 */
static void DMMotorDecode(CANInstance *_instance)
{
    uint8_t          *rxbuff = _instance->rx_buff;
    DMMotorInstance  *motor  = (DMMotorInstance *)_instance->id;
    DM_Motor_Measure_s *m    = &motor->measure;

    /* 1. 喂狗 */
    DaemonReload(motor->motor_daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    /* 2. 解析编码器 */
    m->last_ecd = m->ecd;
    m->ecd      = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    m->angle_single_round = DM_ECD_ANGLE_COEF * (float)m->ecd;

    /* 3. 解析速度 (rpm → 度/秒), 低通滤波 */
    m->speed_aps = (1.0f - DM_SPEED_SMOOTH_COEF) * m->speed_aps
                 + RPM_2_ANGLE_PER_SEC * DM_SPEED_SMOOTH_COEF
                 * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));

    /* 4. 解析电流 (mA → A), 低通滤波 */
    m->real_current = (1.0f - DM_CURRENT_SMOOTH_COEF) * m->real_current
                    + DM_CURRENT_SMOOTH_COEF
                    * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));

    /* 5. 温度 */
    m->temperature = rxbuff[6];

    /* 6. 多圈角度计算 (假设两次采样间转角 < 180°) */
    if (m->ecd - m->last_ecd > DM_HALF_ECD_RANGE)
        m->total_round--;
    else if (m->ecd - m->last_ecd < -DM_HALF_ECD_RANGE)
        m->total_round++;

    m->total_angle = m->total_round * 360.0f + m->angle_single_round;
}

/* ======================== 私有: 离线回调 ======================== */

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    uint16_t can_bus = (motor->motor_can_instance->can_handle == &hcan1) ? 1 : 2;
    LOGWARNING("[dm_motor] Motor lost, can_bus=%d, tx_id=%d",
               can_bus, motor->motor_can_instance->tx_id);

    /* 重新发送使能指令, 尝试恢复 */
    DMMotorSetCommand(motor, DM_CMD_MOTOR_MODE);
}

/* ======================== 私有: 电机分组 (DJI_MODE 专用) ======================== */

/**
 * @brief 根据电机类型和 ID 自动计算 CAN 分组
 *
 * 与 DJI 的 MotorSenderGrouping() 对齐:
 *  - 根据 motor_type 和 tx_id 计算 rx_id
 *  - 分配 sender_group (0-5) 和 message_num (0-3)
 *  - 检测 ID 冲突
 *
 * 分组规则:
 *
 *   M2006_DM / M3508_DM:
 *     tx_id 1-4 → sender_group 1(can1) / 4(can2), rx_id = 0x200 + id
 *     tx_id 5-8 → sender_group 0(can1) / 3(can2), rx_id = 0x200 + (id-4)
 *
 *   GM6020_DM:
 *     tx_id 1-4 → sender_group 0(can1) / 3(can2), rx_id = 0x204 + id
 *     tx_id 5-8 → sender_group 2(can1) / 5(can2), rx_id = 0x204 + (id-4)
 */
static void MotorSenderGrouping(DMMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id       = config->tx_id - 1; // 下标从 0 开始
    uint8_t motor_send_num;   // 组内编号 0-3
    uint8_t motor_grouping;   // 组号 0-5

    switch (motor->motor_type) {

    case J4310:
    case J3507:
        if (motor_id < 4) {
            motor_send_num = motor_id;
            motor_grouping = (config->can_handle == &hcan1) ? 1 : 4;
        } else {
            motor_send_num = motor_id - 4;
            motor_grouping = (config->can_handle == &hcan1) ? 0 : 3;
        }
        config->rx_id = 0x200 + motor_id + 1;
        break;

    case J4340:
        if (motor_id < 4) {
            motor_send_num = motor_id;
            motor_grouping = (config->can_handle == &hcan1) ? 0 : 3;
        } else {
            motor_send_num = motor_id - 4;
            motor_grouping = (config->can_handle == &hcan1) ? 2 : 5;
        }
        config->rx_id = 0x204 + motor_id + 1;
        break;

    default:
        while (1)
            LOGERROR("[dm_motor] Unsupported motor type for grouping!");
        return;
    }

    /* 记录分组信息 */
    sender_enable_flag[motor_grouping] = 1;
    motor->message_num  = motor_send_num;
    motor->sender_group = motor_grouping;

    /* ID 冲突检测 */
    for (size_t i = 0; i < idx; i++) {
        if (dm_motor_instance[i]->motor_can_instance->can_handle == config->can_handle
            && dm_motor_instance[i]->motor_can_instance->rx_id == config->rx_id) {
            uint16_t can_bus = (config->can_handle == &hcan1) ? 1 : 2;
            LOGERROR("[dm_motor] ID crash! id=%d, can_bus=%d", config->rx_id, can_bus);
            while (1); // 死循环便于调试定位
        }
    }
}

/* ======================== 外部接口: 初始化 ======================== */

/**
 * @brief 初始化达妙电机实例
 *
 * 流程 (与 DJIMotorInit 对齐):
 *  1. malloc 实例 + memset 清零
 *  2. 记录 motor_type 和 control_mode
 *  3. 复制 motor_settings / 初始化 PID (三环 + 陀螺仪角度环)
 *  4. 设置前馈和反馈指针
 *  5. 根据 control_mode 偏移 tx_id
 *  6. DJI_MODE 下调用 MotorSenderGrouping() 自动分组
 *  7. 注册 CAN 回调 + Daemon 离线监控
 *  8. 发送使能命令
 *  9. 加入全局实例数组
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config, DMControl_Mode_e control_mode)
{
    /* --- 1. 分配实例 --- */
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    /* --- 2. 基本属性 --- */
    motor->motor_type   = config->motor_type;
    motor->control_mode = control_mode;

    /* --- 3. 控制配置 --- */
    motor->motor_settings = config->controller_setting_init_config;

    /* 初始化标准三环 PID (复用 Motor_Controller_s) */
    PIDInit(&motor->motor_controller.current_PID,
            &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID,
            &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID,
            &config->controller_param_init_config.angle_PID);

    /* 初始化陀螺仪角度环 PID (云台专用, 与编码器 angle_PID 独立) */
    PIDInit(&motor->gyro_angle_PID,
            &config->controller_param_init_config.gyro_PID);

    /* --- 4. 前馈与其他反馈指针 --- */
    motor->motor_controller.other_angle_feedback_ptr =
        config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr =
        config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.speed_feedforward_ptr =
        config->controller_param_init_config.speed_feedforward_ptr;
    motor->motor_controller.current_feedforward_ptr =
        config->controller_param_init_config.current_feedforward_ptr;

    /* --- 5. 根据控制模式偏移 tx_id --- */
    switch (control_mode) {
    case DM_MIT_MODE:    /* tx_id 不变 */                              break;
    case DM_POSVEL_MODE: config->can_init_config.tx_id += 0x100;       break;
    case DM_VEL_MODE:    config->can_init_config.tx_id += 0x200;       break;
    case DM_DJI_MODE:    config->can_init_config.tx_id += 0x3FE;       break;
    default:
        while (1) LOGERROR("[dm_motor] Undefined control mode!");
    }

    /* --- 6. DJI_MODE 下自动分组, 其他模式电机独立发送 --- */
    if (control_mode == DM_DJI_MODE) {
        MotorSenderGrouping(motor, &config->can_init_config);
    } else {
        motor->sender_group = 0xFF; // 标记为无效分组
        motor->message_num  = 0;
    }

    /* --- 7. 注册 CAN 回调 + Daemon --- */
    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instance = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s daemon_conf = {
        .callback     = DMMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 10, // 100ms 超时 (1kHz 控制频率下 10 个周期)
    };
    motor->motor_daemon = DaemonRegister(&daemon_conf);

    /* --- 8. 使能电机 --- */
    DMMotorEnable(motor);
    DMMotorSetCommand(motor, DM_CMD_MOTOR_MODE);
    DWT_Delay(0.1);

    /* --- 9. 加入全局数组 --- */
    dm_motor_instance[idx++] = motor;
    return motor;
}

/* ======================== 外部接口: 设定参考值 ======================== */

/**
 * @brief 设定电机目标值 (与 DJIMotorSetRef 对齐)
 *
 * 单一参考值, 存入 motor_controller.pid_ref.
 * 对于 MIT/POSVEL/VEL 模式, pid_ref 在 DMMotorControl() 中直接映射为物理量.
 * 对于 DJI_MODE, pid_ref 串级流经角度环→速度环→电流环.
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    if (motor == NULL) return;
    motor->motor_controller.pid_ref = ref;
}

/* ======================== 外部接口: 切换反馈来源 ======================== */

/**
 * @brief 切换反馈数据来源 (与 DJIMotorChangeFeed 对齐)
 *
 * 典型用法: 云台从编码器模式切换到陀螺仪模式
 *   DMMotorChangeFeed(motor_yaw, ANGLE_LOOP, OTHER_FEED);
 *   DMMotorChangeFeed(motor_yaw, SPEED_LOOP, OTHER_FEED);
 *   // 同时需要设置:
 *   motor_yaw->motor_controller.other_angle_feedback_ptr = &imu->Yaw;
 *   motor_yaw->motor_controller.other_speed_feedback_ptr  = &imu->Gyro[2];
 */
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop,
                       Feedback_Source_e source)
{
    if (motor == NULL) return;

    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = source;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = source;
    else
        LOGERROR("[dm_motor] Invalid loop type for ChangeFeed");
}

/* ======================== 外部接口: 修改外层闭环 ======================== */

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    if (motor == NULL) return;
    motor->motor_settings.outer_loop_type = outer_loop;
}

/* ======================== 外部接口: 启停控制 ======================== */

void DMMotorEnable(DMMotorInstance *motor)
{
    if (motor == NULL) return;
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)
{
    if (motor == NULL) return;
    motor->stop_flag = MOTOR_STOP;
}

/* ======================== 外部接口: 辅助功能 ======================== */

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    if (motor == NULL) return;
    DMMotorSetCommand(motor, DM_CMD_ZERO_POSITION);
    DWT_Delay(0.1);
}

void DMMotorShootFlag(DMMotorInstance *motor, uint8_t shoot_flag)
{
    if (motor == NULL) return;
    motor->shoot_flag = shoot_flag;
}

/* ======================== 核心: 集中式电机控制 ======================== */

/**
 * @brief 集中式 DM 电机控制函数 (与 DJIMotorControl 对齐)
 *
 * 设计要点:
 *  1. 遍历所有已注册 DM 电机
 *  2. 根据 control_mode 派发不同控制逻辑
 *  3. DJI_MODE 下使用串级 PID 框架 (与 DJI 完全相同)
 *  4. 统一分组发送, 防止空帧
 *
 * 调用方式:
 *   void MotorControlTask(void) {
 *       DJIMotorControl();
 *       DMMotorControl();  // ← 新增, 以 1kHz 调用
 *   }
 */
void DMMotorControl(void)
{
    DMMotorInstance          *motor;
    Motor_Control_Setting_s  *setting;
    Motor_Controller_s       *ctrl;
    DM_Motor_Measure_s       *measure;

    float  pid_measure;
    float  pid_ref;
    int16_t set;

    /* ===== 遍历所有 DM 电机 ===== */
    for (size_t i = 0; i < idx; i++) {
        motor   = dm_motor_instance[i];
        setting = &motor->motor_settings;
        ctrl    = &motor->motor_controller;
        measure = &motor->measure;

        pid_ref = ctrl->pid_ref; // 快照, 防止计算中被修改

        /* 反转处理 */
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        /* ===== 根据控制模式派发 ===== */
        switch (motor->control_mode) {

        /* ---------- MIT 模式: 发送力矩目标 ---------- */
        case DM_MIT_MODE: {
            float torque = pid_ref; // pid_ref 即为力矩目标 (N·m)
            LIMIT_MIN_MAX(torque, DM_T_MIN, DM_T_MAX);

            DMMotor_Send_MIT_s mit;
            mit.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            mit.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            mit.torque_des   = float_to_uint(torque, DM_T_MIN, DM_T_MAX, 12);
            mit.Kp = 0;
            mit.Kd = 0;

            if (motor->stop_flag == MOTOR_STOP)
                mit.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

            /* 填入 CAN 发送缓冲区 */
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
            break;
        }

        /* ---------- 位置速度模式: 发送位置+速度 ---------- */
        case DM_POSVEL_MODE: {
            float pos = pid_ref;
            float vel = 0; // 速度目标通过前馈指针设定
            if (ctrl->speed_feedforward_ptr)
                vel = *ctrl->speed_feedforward_ptr;

            LIMIT_MIN_MAX(pos, DM_P_MIN, DM_P_MAX);
            LIMIT_MIN_MAX(vel, DM_V_MIN, DM_V_MAX);

            if (motor->stop_flag == MOTOR_STOP) {
                pos = 0; vel = 0;
            }

            CANInstance *can = motor->motor_can_instance;
            memcpy(can->tx_buff,     &pos, 4);
            memcpy(can->tx_buff + 4, &vel, 4);
            CANTransmit(can, 1);
            break;
        }

        /* ---------- 速度模式: 发送速度 ---------- */
        case DM_VEL_MODE: {
            float vel = pid_ref;
            LIMIT_MIN_MAX(vel, DM_V_MIN, DM_V_MAX);

            if (motor->stop_flag == MOTOR_STOP)
                vel = 0;

            CANInstance *can = motor->motor_can_instance;
            memcpy(can->tx_buff, &vel, 4);
            CANTransmit(can, 1);
            break;
        }

        /* ---------- DJI 兼容模式: MCU 侧串级 PID ---------- */
        case DM_DJI_MODE:
        {
            /* ---- 角度环 (ANGLE_LOOP) ---- */
            if ((setting->close_loop_type & ANGLE_LOOP)
                && setting->outer_loop_type == ANGLE_LOOP) {

                if (setting->angle_feedback_source == OTHER_FEED) {
                    /* 陀螺仪模式: 使用 gyro_angle_PID + 陀螺仪角度反馈 */
                    pid_measure = (ctrl->other_angle_feedback_ptr)
                                ? *ctrl->other_angle_feedback_ptr
                                : measure->relative_angle_gyro;
                    pid_ref = PIDCalculate(&motor->gyro_angle_PID,
                                           pid_measure, pid_ref);
                } else {
                    /* 编码器模式: 使用 angle_PID + 编码器角度反馈 */
                    pid_measure = measure->total_angle;
                    pid_ref = PIDCalculate(&ctrl->angle_PID,
                                           pid_measure, pid_ref);
                }
            }

            /* ---- 速度环 (SPEED_LOOP) ---- */
            if ((setting->close_loop_type & SPEED_LOOP)
                && (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {

                /* 速度前馈 */
                if (setting->feedforward_flag & SPEED_FEEDFORWARD
                    && ctrl->speed_feedforward_ptr)
                    pid_ref += *ctrl->speed_feedforward_ptr;

                if (setting->speed_feedback_source == OTHER_FEED) {
                    /* 陀螺仪模式: 使用 IMU 角速度 */
                    pid_measure = (ctrl->other_speed_feedback_ptr)
                                ? *ctrl->other_speed_feedback_ptr
                                : measure->gyro_rate;
                } else {
                    /* 编码器模式: 使用电机编码器速度 */
                    pid_measure = measure->speed_aps;
                }
                pid_ref = PIDCalculate(&ctrl->speed_PID, pid_measure, pid_ref);
            }

            /* ---- 电流环 (CURRENT_LOOP) ---- */
            if (setting->feedforward_flag & CURRENT_FEEDFORWARD
                && ctrl->current_feedforward_ptr)
                pid_ref += *ctrl->current_feedforward_ptr;

            if (setting->close_loop_type & CURRENT_LOOP) {
                pid_ref = PIDCalculate(&ctrl->current_PID,
                                       measure->real_current, pid_ref);
            }

            /* 反馈反向 */
            if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                pid_ref *= -1.0f;

            /* 停止处理 */
            if (motor->stop_flag == MOTOR_STOP)
                pid_ref = 0;

            set = (int16_t)pid_ref;

            /* 填入分组发送缓冲区 */
            uint8_t group = motor->sender_group;
            uint8_t num   = motor->message_num;
            sender_assignment[group].tx_buff[2 * num]     = (uint8_t)(set >> 8);
            sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff);

            /* 保持电机使能 (每周期发送命令, 防止自动失能) */
            DMMotorSetCommand(motor, DM_CMD_MOTOR_MODE);
            break;
        }

        default:
            break;
        }
    }

    /* ===== 统一发送分组报文 (仅 DJI_MODE 的电机需要) ===== */
    for (size_t i = 0; i < 6; i++) {
        if (sender_enable_flag[i]) {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}

/* ======================== 外部接口: 控制初始化 ======================== */

/**
 * @brief DM 电机控制初始化
 *
 * 与旧版 DMMotorControlInit() 完全不同:
 *  - 不再为每个电机创建 FreeRTOS 任务
 *  - 仅做校验和日志输出
 *  - 实际控制由 DMMotorControl() 在 MotorControlTask 中统一执行
 */
void DMMotorControlInit(void)
{
    if (idx == 0) {
        LOGINFO("[dm_motor] No DM motor registered, skip init.");
        return;
    }

    LOGINFO("[dm_motor] %d motor(s) registered, centralized control ready.", idx);

    /* 打印注册信息, 便于调试 */
    for (size_t i = 0; i < idx; i++) {
        DMMotorInstance *m = dm_motor_instance[i];
        uint16_t can_bus = (m->motor_can_instance->can_handle == &hcan1) ? 1 : 2;
        const char *mode_str[] = {"MIT", "POSVEL", "VEL", "DJI"};
        LOGINFO("[dm_motor] [%d] type=%d mode=%s can=%d tx_id=%d rx_id=%d",
                i, m->motor_type,
                mode_str[m->control_mode],
                can_bus,
                m->motor_can_instance->tx_id,
                m->motor_can_instance->rx_id);
    }
}