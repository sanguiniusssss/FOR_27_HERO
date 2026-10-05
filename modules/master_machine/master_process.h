/**
 * @file master_process.h
 * @brief  与视觉上位机(NUC)的通信模块, 采用 TJ 协议(USB 虚拟串口)
 *         帧头 'S''P', 整包 CRC16 校验
 * @note   协议源自 gimbal_and_trigger 工程, 在这里复用其 NUC 端
 */
#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include <stdint.h>
#include "bsp_usb.h"

/* 使用枚举值, 与 NUC 端保持一致 */
typedef enum
{
    COLOR_NONE = 0,
    COLOR_BLUE = 1,
    COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

#pragma pack(1)
/* 主控 -> NUC : 43 字节 */
typedef struct
{
    uint8_t head[2]; // 'S''P'
    uint8_t mode;    // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];      // 四元数 wxyz
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} TJ_T_t;

/* NUC -> 主控 */
typedef struct
{
    uint8_t head[2]; // 'S''P'
    uint8_t mode;    // 0: 不控制, 1: 只自瞄(不开火), 2: 自瞄+开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} TJ_R_t;
#pragma pack()

/* 解析后暴露给应用层使用的接收数据 */
typedef struct
{
    uint8_t state; // 来自 TJ_R.mode: 0 不控制, >0 自瞄中
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
} Vision_Recv_s;

/* 主控发送给 NUC 的数据 */
typedef struct
{
    Enemy_Color_e enemy_color;
    Work_Mode_e work_mode;
    Bullet_Speed_e bullet_speed;

    float yaw;
    float pitch;
    float roll;
} Vision_Send_s;

/**
 * @brief  初始化与视觉的 USB 通信, 返回接收数据结构体指针
 */
Vision_Recv_s *VisionInit(void);

/**
 * @brief  获取视觉接收数据(供云台等应用读取)
 */
Vision_Recv_s *VisionGetRecvData(void);

/**
 * @brief  发送姿态和状态给 NUC
 */
void VisionSend(void);

/**
 * @brief  设置发送给 NUC 的标志位
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief  设置发送给 NUC 的姿态角
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif // !MASTER_PROCESS_H
