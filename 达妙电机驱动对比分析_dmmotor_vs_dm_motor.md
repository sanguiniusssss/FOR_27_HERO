# 达妙电机驱动双版本深度对比分析

## —— dmmotor (v1 beta) vs dm_motor (v2.0)

---

> **文档作者**: 基于源码分析自动生成
> **分析日期**: 2026-08-06
> **项目路径**: `C:\Users\Sanguinior\Desktop\FOR_27_HERO`
> **涉及文件**:
> - 旧版: `modules/motor/DMmotor/dmmotor.c` (362行) + `dmmotor.h` (173行)
> - 新版: `modules/motor/DMmotor/dm_motor.c` (649行) + `dm_motor.h` (262行)
> - 公用: `modules/motor/motor_def.h`, `modules/motor/motor_task.c`
> - 关键状态: **两个版本在同一目录下共存，Makefile 同时编译两者，存在符号冲突**

---

## 目录

1. [版本基本信息](#1-版本基本信息)
2. [架构设计对比](#2-架构设计对比)
3. [数据结构对比](#3-数据结构对比)
4. [API 接口对比](#4-api-接口对比)
5. [控制模式与PID对比](#5-控制模式与pid对比)
6. [CAN通信与分组对比](#6-can通信与分组对比)
7. [编码器与角度计算对比](#7-编码器与角度计算对比)
8. [电机类型支持对比](#8-电机类型支持对比)
9. [初始化流程对比](#9-初始化流程对比)
10. [核心控制回路对比](#10-核心控制回路对比)
11. [守护进程与离线监控对比](#11-守护进程与离线监控对比)
12. [代码质量与健壮性对比](#12-代码质量与健壮性对比)
13. [当前兼容性冲突分析](#13-当前兼容性冲突分析)
14. [迁移指南](#14-迁移指南)
15. [总结表](#15-总结表)

---

## 1. 版本基本信息

| 维度 | dmmotor (旧版) | dm_motor (新版) |
|------|---------------|----------------|
| **文件名** | `dmmotor.c` / `dmmotor.h` | `dm_motor.c` / `dm_motor.h` |
| **版本号** | beta | 2.0 |
| **最后修改日期** | 2025-05-01 | 2026-08-05 |
| **作者** | Weedy | neozng / Weedy (重构) |
| **主头文件** | `#include "dmmotor.h"` | `#include "dm_motor.h"` |
| **版权声明** | Copyright (c) 2022 | Copyright (c) 2022-2026 |
| **代码行数 (.c)** | 362行 | 649行 |
| **代码行数 (.h)** | 173行 | 262行 |
| **最大电机数** | `DM_MOTOR_CNT = 4` | `DM_MOTOR_CNT = 8` |
| **状态** | 遗留代码，应废弃 | 正式版本，应使用 |

---

## 2. 架构设计对比

这是两个版本**最根本的区别**。

### 2.1 控制任务架构

```
┌─────────────────────────────────────────────────────────────────┐
│                     旧版 (dmmotor)                               │
│                                                                 │
│  DMMotorControlInit()                                           │
│       │                                                         │
│       ├──→ osThreadCreate("dm0", DMMotorTask, motor[0])         │
│       ├──→ osThreadCreate("dm1", DMMotorTask, motor[1])         │
│       ├──→ osThreadCreate("dm2", DMMotorTask, motor[2])         │
│       └──→ osThreadCreate("dm3", DMMotorTask, motor[3])         │
│                                                                 │
│  每个电机 = 1个独立的 FreeRTOS 任务                              │
│  4电机 × 128字节栈 = 512字节 RAM                                │
│  每个任务内部 while(1) { PID计算 → CAN发送 → osDelay(1) }       │
│                                                                 │
│  问题:                                                          │
│  - 8电机时需要 8个任务 = 1024字节栈                              │
│  - 任务调度开销大 (4-8次上下文切换/周期)                         │
│  - 各电机控制不同步, 可能产生相位差                               │
│  - 无法统一管理发送时序                                          │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                     新版 (dm_motor)                              │
│                                                                 │
│  MotorControlTask()  (1kHz, 在 motor_task.c 中)                  │
│       │                                                         │
│       ├──→ DJIMotorControl()    // DJI 电机遍历                  │
│       └──→ DMMotorControl()     // DM 电机遍历  ← 集中式!        │
│                │                                                │
│                └──→ for(i=0; i<idx; i++) {                      │
│                     switch(control_mode) {                       │
│                       MIT_MODE → 打包→直接CAN发送                │
│                       POSVEL → 打包→直接CAN发送                  │
│                       VEL_MODE → 打包→直接CAN发送                │
│                       DJI_MODE → 串级PID→分组缓冲→统一CAN发送    │
│                     }                                           │
│                    }                                            │
│                                                                 │
│  0个额外 FreeRTOS 任务                                          │
│  所有 DM 电机在同一函数中顺序处理                                │
│  与 DJI 电机控制函数在同一个 1kHz 任务中同步调用                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 架构影响分析

| 维度 | 旧版 (每电机一任务) | 新版 (集中式) |
|------|-------------------|--------------|
| **RTOS 任务数** | N个 (N=电机数) | 0个额外 |
| **栈内存消耗** | N × 128 bytes | 0 (共享 MotorControlTask 栈) |
| **上下文切换** | 每周期 N 次 | 0次额外 |
| **控制同步性** | 各电机异步, 可能有相位差 | 严格同步, 同一函数内顺序执行 |
| **CPU 缓存友好度** | 差 (任务切换刷新缓存) | 好 (连续内存访问) |
| **与 DJI 电机的一致性** | 不一致 (DJI 用集中式) | **完全一致** |
| **代码可维护性** | 独立修改每个任务逻辑 | 统一控制循环, 修改一处即可 |

---

## 3. 数据结构对比

### 3.1 测量数据结构: `DM_Motor_Measure_s`

这是两个版本中**字段差异最大**的结构体。新版进行了大幅精简和重命名。

```
旧版 (dmmotor.h)                          新版 (dm_motor.h)
══════════════════════════════════        ══════════════════════════════════
uint8_t  id;                     ←──      删除 (移到实例层 motor_type)
uint8_t  state;                  ←──      删除 (DM 电机无此反馈)
uint8_t  init_flag;              ──→      uint8_t  init_flag;        ✓ 保留
uint8_t  temperature;            ──→      uint8_t  temperature;      ✓ 保留

uint16_t ecd;                    ──→      uint16_t ecd;              ✓ 保留
uint16_t last_ecd;               ──→      uint16_t last_ecd;         ✓ 保留
uint16_t offset_ecd;             ──→      uint16_t offset_ecd;       ✓ 保留
int32_t  relative_ecd;           ──→      int32_t  relative_ecd;     ✓ 保留
float    angle_single_round;     ──→      float    angle_single_round; ✓ 保留
int32_t  total_round;            ──→      int32_t  total_round;      ✓ 保留
float    total_angle;            ──→      float    total_angle;       ✓ 保留
float    init_angle;             ──→      float    init_angle;        ✓ 保留
float    relative_angle;         ──→      float    relative_angle;    ✓ 保留

float    gyro_angle;             ──→      float    gyro_angle;        ✓ 保留
float    relative_angle_gyro;    ──→      float    relative_angle_gyro; ✓ 保留
float    gyro;                   ──→      float    gyro_rate;         ✗ 重命名
float    offest_angle;           ──→      float    gyro_offset_angle; ✗ 重命名+拼写修正
float    accel;                  ──→      float    gyro_accel;        ✗ 重命名+语义更清晰

float    velocity;               ←──      删除 (MIT模式专用, DJI模式不需要)
float    speed_aps;              ──→      float    speed_aps;         ✓ 保留
float    real_current;           ──→      float    real_current;      ✓ 保留
float    torque;                 ←──      删除 (MIT模式专用)
float    position;               ←──      删除 (MIT模式专用)
float    last_position;          ←──      删除 (MIT模式专用)
float    T_Mos;                  ←──      删除 (未使用)
float    T_Rotor;                ←──      删除 (未使用)
```

**关键变更分析:**

1. **删除 `id`, `state`**: 旧版把 `id` 和 `state` 放在 measure 中不合理——这是电机属性而非测量值。新版将电机类型识别移到了实例层的 `motor_type` 字段。

2. **删除 MIT 专用字段** (`velocity`, `torque`, `position`, `last_position`): 旧版的 `DM_Motor_Measure_s` 同时承载了 MIT 模式和 DJI 模式的反馈数据，导致结构体臃肿。MIT 模式下电机自身完成闭环，MCU 不需要这些数据做 PID 计算。新版做了合理的职责分离。

3. **删除未使用的温度字段** (`T_Mos`, `T_Rotor`): 旧版定义了但代码中从未使用。

4. **陀螺仪字段重命名**: 
   - `gyro` → `gyro_rate` (明确是角速率而非角度)
   - `accel` → `gyro_accel` (明确是加速度计数据)
   - `offest_angle` → `gyro_offset_angle` (修正拼写错误 `offest`→`offset`，增加 `gyro_` 前缀统一命名)

### 3.2 电机实例结构: `DMMotorInstance`

```
旧版 (dmmotor.h)                          新版 (dm_motor.h)
══════════════════════════════════        ══════════════════════════════════
                                         Motor_Type_e motor_type;      ← 新增
DMControl_Mode_e control_mode;    ──→    DMControl_Mode_e control_mode; ✓ 保留

DM_Motor_Measure_s measure;       ──→    DM_Motor_Measure_s measure;   ✓ (结构体内容不同)

Motor_Control_Setting_s                                              ✓ 保留
    motor_settings;                ──→    Motor_Control_Setting_s motor_settings;
                                         Motor_Controller_s            ← 新增 (集中式PID框架)
                                             motor_controller;

PIDInstance gyro_PID;             ──→    PIDInstance gyro_angle_PID;  ✗ 移到 motor_controller
PIDInstance speed_PID;            ──→    (在 motor_controller 中)     ✗ 统一管理
PIDInstance angle_PID;            ──→    (在 motor_controller 中)     ✗ 统一管理
PIDInstance current_PID;          ──→    (在 motor_controller 中)     ✗ 统一管理

float *other_angle_feedback_ptr;  ──→    (在 motor_controller 中)     ✗ 统一管理
float *other_speed_feedback_ptr;  ──→    (在 motor_controller 中)     ✗ 统一管理
float *speed_feedforward_ptr;     ──→    (在 motor_controller 中)     ✗ 统一管理
                                       float *gyro_feedback_ptr;      ← 新增 (陀螺仪角度指针)

float pid_ref[3];                 ──→    (在 motor_controller 中:      ✗ 数组→标量
                                         motor_controller.pid_ref)       串级流转
float raw_gyro;                   ←──    删除 (未有效使用)
float relative_gyro_yaw;          ←──    删除 (未有效使用)
float relative_gyro_pitch;        ←──    删除 (未有效使用)

Motor_Working_Type_e stop_flag;   ──→    Motor_Working_Type_e stop_flag; ✓ 保留

CANInstance *motor_can_instace;   ──→    CANInstance *motor_can_instance; ✓ (修正拼写)
DaemonInstance *motor_daemon;     ──→    DaemonInstance *motor_daemon;  ✓ 保留
uint32_t lost_cnt;                ←──    删除
                                       uint32_t feed_cnt;             ← 新增 (替代)
                                       float dt;                      ← 新增 (控制周期)

uint8_t sender_group;             ──→    uint8_t sender_group;         ✓ 保留
uint8_t message_num;              ──→    uint8_t message_num;          ✓ 保留

uint8_t maker_flag;               ←──    删除 (改为通过 Feedback_Source_e 判断)
uint8_t extern_flag;              ←──    删除 (不再需要区分 yaw/pitch)
uint8_t shoot_flag_dm;            ──→    uint8_t shoot_flag;           ✗ 重命名

Motor_Controller_s                                                     ← 旧版也有但未有效使用
    motor_controller;              ──→    (新版重新设计)
```

**关键变更分析:**

1. **PID 控制器集中管理**: 旧版将 `gyro_PID`, `speed_PID`, `angle_PID`, `current_PID` 作为 `DMMotorInstance` 的直接字段。新版将它们统一放入 `Motor_Controller_s` 结构体，与 DJI 电机驱动 (`DJIMotorInstance`) 完全一致。

2. **`pid_ref` 从数组变标量**: 这是架构升级的核心。
   - 旧版: `pid_ref[3]` — 位置/速度/电流三个独立目标值，需要应用层分别设置
   - 新版: `motor_controller.pid_ref` — **单一标量**，在串级 PID 中自动流转
   ```
   旧版:  app → pid_ref[0]=角度, pid_ref[1]=速度, pid_ref[2]=电流
   新版:  app → pid_ref=角度 → 角度环输出覆盖 pid_ref → 速度环用新值 → ...
   ```

3. **删除 `maker_flag` 和 `extern_flag`**: 旧版用两个标志位区分编码器/陀螺仪模式和 yaw/pitch 轴，逻辑分散且容易出错。新版通过 `Feedback_Source_e` 枚举 (`MOTOR_FEED` / `OTHER_FEED`) 和 `Motor_Control_Setting_s` 中的 `angle_feedback_source` / `speed_feedback_source` 来统一管理反馈来源。

4. **新增 `motor_type`**: 旧版没有记录电机硬件类型，分组发送时无法区分 J4310/J3507/J4340。新版在实例层记录，支持自动分组 (`MotorSenderGrouping`)。

5. **新增 `dt` (控制周期)**: 新版在 CAN 解码回调中通过 `DWT_GetDeltaT(&motor->feed_cnt)` 计算实际控制周期，可用于自适应 PID 参数调整（虽然当前版本尚未使用）。

### 3.3 发送数据结构

两个版本在 MIT/POSVEL/VEL 模式的 CAN 发送数据结构上**完全一致**：

```c
// MIT 模式发送 (两版本一致)
typedef struct {
    uint16_t position_des;  // 16位
    uint16_t velocity_des;  // 12位
    uint16_t torque_des;    // 12位
    uint16_t Kp;            // 12位
    uint16_t Kd;            // 12位
} DMMotor_Send_MIT_s;

// POSVEL 模式发送 (两版本一致, union+float 直接 memcpy)
typedef struct {
    union { float position_des; uint8_t data[4]; } p_des;
    union { float velocity_des; uint8_t data[4]; } v_des;
} DMMotor_Send_PosVel_s;

// VEL 模式发送 (两版本一致)
typedef struct {
    union { float velocity_des; uint8_t data[4]; } v_des;
} DMMotor_Send_Vel_s;
```

旧版额外定义了 `DMMotor_Send_DJI_s`（未被实际使用），新版将其删除。

---

## 4. API 接口对比

### 4.1 函数签名对比

| 功能 | 旧版 (dmmotor) | 新版 (dm_motor) |
|------|---------------|----------------|
| **初始化** | `DMMotorInstance *DMMotorInit(config, control_mode)` | `DMMotorInstance *DMMotorInit(config, control_mode)` ✓ 一致 |
| **设目标值** | `void DMMotorSetRef(motor, ref1, ref2, ref3, maker_flag, extern_flag)` | `void DMMotorSetRef(motor, ref)` |
| **控制执行** | `void DMMotorTask(argument)` (每电机任务) | `void DMMotorControl(void)` (集中式) |
| **控制初始化** | `void DMMotorControlInit()` (创建任务) | `void DMMotorControlInit(void)` (仅校验+日志) |
| **使能** | `void DMMotorEnable(motor)` | `void DMMotorEnable(motor)` ✓ 一致 |
| **停止** | `void DMMotorStop(motor)` | `void DMMotorStop(motor)` ✓ 一致 |
| **校零** | `void DMMotorCaliEncoder(motor)` | `void DMMotorCaliEncoder(motor)` ✓ 一致 |
| **外环切换** | `void DMMotorOuterLoop(motor, type)` | `void DMMotorOuterLoop(motor, outer_loop)` ✓ 一致 |
| **射击标志** | `void DMMotorShootFlag(motor, shoot_flag)` | `void DMMotorShootFlag(motor, shoot_flag)` ✓ 一致 |
| **发送指令** | `static DMMotorSetMode(cmd, motor)` (私有) | `void DMMotorSetCommand(motor, cmd)` (公开) |
| **切换反馈源** | 无 | `void DMMotorChangeFeed(motor, loop, source)` ← 新增 |
| **空指针检查** | ❌ 无 | ✅ 每个函数都有 `if(motor==NULL) return` |

### 4.2 `DMMotorSetRef` 参数变化（重点！）

这是**应用层迁移时最需要注意的 API 变化**：

```c
// ═══════════ 旧版 ═══════════
// 需要传递 6 个参数！
void DMMotorSetRef(DMMotorInstance *motor, 
                   float ref1,          // 位置/角度目标
                   float ref2,          // 速度目标
                   float ref3,          // 电流/扭矩目标
                   uint8_t maker_flag,  // 0=编码器模式, 1=陀螺仪模式
                   uint8_t extern_flag) // 0=yaw, 1=pitch

// ═══════════ 新版 ═══════════
// 只需 2 个参数！与 DJIMotorSetRef 完全一致
void DMMotorSetRef(DMMotorInstance *motor, 
                   float ref)           // 目标值(根据 outer_loop_type 解释)
```

**旧版调用示例** (gimbal.c 当前写法):

```c
// Yaw 轴 - 编码器模式
DMMotorSetRef(motor_yaw, 
    target_angle,    // ref1: 角度目标
    0,               // ref2: 不用
    0,               // ref3: 不用
    0,               // maker_flag: 编码器模式
    0);              // extern_flag: yaw

// Pitch 轴 - 陀螺仪模式  
DMMotorSetRef(motor_pitch,
    target_angle,    // ref1: 角度目标
    0,               // ref2: 不用
    0,               // ref3: 不用
    1,               // maker_flag: 陀螺仪模式
    1);              // extern_flag: pitch
```

**新版调用示例** (迁移后):

```c
// Yaw 轴
DMMotorOuterLoop(motor_yaw, ANGLE_LOOP);           // 设定外环为角度环
DMMotorChangeFeed(motor_yaw, ANGLE_LOOP, MOTOR_FEED); // 编码器反馈
DMMotorSetRef(motor_yaw, target_angle);             // 只需一个参数!

// Pitch 轴 - 陀螺仪模式
DMMotorOuterLoop(motor_pitch, ANGLE_LOOP);
DMMotorChangeFeed(motor_pitch, ANGLE_LOOP, OTHER_FEED); // IMU反馈
motor_pitch->motor_controller.other_angle_feedback_ptr = &imu->Pitch;
DMMotorSetRef(motor_pitch, target_angle);
```

### 4.3 新增 `DMMotorChangeFeed` 函数（新版独有）

```c
/**
 * @brief 切换反馈数据来源 (与 DJIMotorChangeFeed 对齐)
 *
 * @param motor  电机实例指针
 * @param loop   要切换的闭环 (ANGLE_LOOP / SPEED_LOOP)
 * @param source 反馈来源 (MOTOR_FEED / OTHER_FEED)
 */
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop,
                       Feedback_Source_e source);
```

这个函数替代了旧版的 `maker_flag` 机制：
- 旧版: 每次 `DMMotorSetRef` 都要传 `maker_flag`，容易忘记或传错
- 新版: 初始化时调用一次 `DMMotorChangeFeed`，之后无需每次设置

### 4.4 新增 `DMMotorSetCommand` 函数（新版独有）

```c
// 旧版: 私有 static 函数，外部无法调用
static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor);

// 新版: 公开 API，应用层可发送控制命令
void DMMotorSetCommand(DMMotorInstance *motor, DMMotor_Command_e cmd);
```

应用层可以使用 `DMMotorSetCommand(motor, DM_CMD_CLEAR_ERROR)` 来清除电机过热错误，旧版做不到这一点。

---

## 5. 控制模式与PID对比

### 5.1 控制模式枚举

| 旧版 | 新版 | 值 |
|------|------|----|
| `MIT_MODE` | `DM_MIT_MODE` | 0 |
| `POSVEL_MODE` | `DM_POSVEL_MODE` | 1 |
| `VEL_MODE` | `DM_VEL_MODE` | 2 |
| `DJI_MODE` | `DM_DJI_MODE` | 3 |

> ⚠️ **重要**: 枚举值名称不同！旧版无 `DM_` 前缀，新版有。如果代码中有 `switch(control_mode) { case MIT_MODE: ... }` 需要改为 `case DM_MIT_MODE:`。

### 5.2 PID 控制器布局对比

```
旧版 PID 布局 (dmmotor):
═══════════════════════════════════════
DMMotorInstance
├── PIDInstance gyro_PID      ← 独立字段
├── PIDInstance speed_PID     ← 独立字段
├── PIDInstance angle_PID     ← 独立字段
├── PIDInstance current_PID   ← 独立字段
└── Motor_Controller_s motor_controller  ← 存在但几乎未使用

问题:
- PID 实例分散, 不与 motor_controller 关联
- motor_controller 形同虚设
- 与 DJI 电机驱动不一致


新版 PID 布局 (dm_motor):
═══════════════════════════════════════
DMMotorInstance
├── Motor_Controller_s motor_controller
│   ├── PIDInstance angle_PID       ← 编码器角度环
│   ├── PIDInstance speed_PID       ← 速度环
│   ├── PIDInstance current_PID     ← 电流环
│   ├── PIDInstance gyro_PID        ← 陀螺仪PID (motor_def.h 中定义)
│   ├── float pid_ref               ← 串级流转变量
│   ├── float *other_angle_feedback_ptr
│   ├── float *other_speed_feedback_ptr
│   ├── float *speed_feedforward_ptr
│   └── float *current_feedforward_ptr
│
└── PIDInstance gyro_angle_PID      ← 陀螺仪角度环 (实例层, 云台专用)

优势:
- 标准三环 PID 统一在 motor_controller 中
- 与 DJIMotorInstance 结构完全对齐
- gyro_angle_PID 独立出来 (因为云台陀螺仪角度环与编码器角度环是不同的控制需求)
```

### 5.3 DJI_MODE 下 PID 串级逻辑对比

```
旧版 DJI_MODE 控制流:
═══════════════════════════════════════
if (maker_flag == 1) {              // 陀螺仪模式
    if (extern_flag == 0) {         // yaw 轴
        angle_err → gyro_PID → out1
        gyro_rate → speed_PID → out2
        → set
    }
    if (extern_flag == 1) {         // pitch 轴
        angle_err → gyro_PID → out1
        gyro_rate → speed_PID → out2
        (+ 发射前馈)
        → set_P
    }
}
if (maker_flag == 0) {              // 编码器模式
    relative_angle → angle_PID → out
    if (extern_flag == 0) → set
    if (extern_flag == 1) → set_P
}
// 问题:
// 1. maker_flag/extern_flag 嵌套 if-else, 逻辑复杂
// 2. yaw/pitch 硬编码分支 (set vs set_P)
// 3. 速度环和角度环之间没有反馈源切换
// 4. 没有电流环!


新版 DJI_MODE 控制流:
═══════════════════════════════════════
// 角度环
if (outer_loop_type == ANGLE_LOOP) {
    if (angle_feedback_source == OTHER_FEED)
        pid_ref = gyro_angle_PID(imu->angle, pid_ref);   // 陀螺仪模式
    else
        pid_ref = angle_PID(measure->total_angle, pid_ref); // 编码器模式
}

// 速度环 (+可选前馈)
if (outer_loop_type & (ANGLE_LOOP|SPEED_LOOP)) {
    if (speed_feedforward_ptr) pid_ref += *speed_feedforward_ptr;
    if (speed_feedback_source == OTHER_FEED)
        pid_ref = speed_PID(imu->gyro_rate, pid_ref);
    else
        pid_ref = speed_PID(measure->speed_aps, pid_ref);
}

// 电流环 (+可选前馈)
if (current_feedforward_ptr) pid_ref += *current_feedforward_ptr;
if (close_loop_type & CURRENT_LOOP)
    pid_ref = current_PID(measure->real_current, pid_ref);

// 优势:
// 1. 清晰的串级结构: 角度→速度→电流, 每一级统一判断反馈来源
// 2. 无 yaw/pitch 硬编码
// 3. 可选的前馈注入点
// 4. 支持电流环闭环!
```

### 5.4 旧版缺少电流环的问题

旧版的 DJI_MODE 中，速度环输出直接作为电流值 (`set = (int16_t)pid_out`)，**没有电流环闭环**。这意味着：

- 输出的电流值完全依赖于速度环 PID 精度
- 无法补偿电机绕组电阻、反电动势等非线性因素
- 新版增加了 `CURRENT_LOOP` 支持，可以对真实电流 (`real_current`) 做闭环校正

---

## 6. CAN通信与分组对比

### 6.1 sender_assignment 定义

两个版本的 CAN 发送分组表**完全一致**（6个分组，CAN1+CAN2 各3组）：

```c
// 两版本相同
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x3FE, ...},  // CAN1 组1
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, ...},  // CAN1 组2
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2FF, ...},  // CAN1 组3
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x3FE, ...},  // CAN2 组1
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, ...},  // CAN2 组2
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2FF, ...},  // CAN2 组3
};
```

### 6.2 分组发送机制

| 维度 | 旧版 | 新版 |
|------|------|------|
| **分组方式** | 手动指定 `sender_group` + `message_num` | `MotorSenderGrouping()` 自动计算 |
| **ID冲突检测** | ❌ 无 | ✅ 遍历已有实例检测冲突 |
| **空帧防止** | ❌ 无标志位 | ✅ `sender_enable_flag[6]` 精确控制 |
| **DJI_MODE 发送** | 每电机独立 CANTransmit | 统一填缓冲 → 遍历6组发送 |
| **MIT/POSVEL/VEL发送** | 直接 CANTransmit | 直接 CANTransmit (与旧版相同) |
| **电机使能维持** | 任务中 osDelay(1) + DMMotorSetMode | DJI_MODE 中每周期调用 |

### 6.3 MotorSenderGrouping（新版独有）

这是新版中最重要的新增私有函数之一：

```c
static void MotorSenderGrouping(DMMotorInstance *motor, CAN_Init_Config_s *config)
{
    // 根据 motor_type 自动选择分组策略:
    //   J4310/J3507 (M2006/M3508 级别):
    //     id 1-4 → group 1(can1)/4(can2), rx_id = 0x200+id
    //     id 5-8 → group 0(can1)/3(can2), rx_id = 0x200+(id-4)
    //   J4340 (GM6020 级别):
    //     id 1-4 → group 0(can1)/3(can2), rx_id = 0x204+id
    //     id 5-8 → group 2(can1)/5(can2), rx_id = 0x204+(id-4)

    // 此外还做:
    // 1. 设置 sender_enable_flag (防止发空帧)
    // 2. ID 冲突检测 (死循环报警)

    sender_enable_flag[motor_grouping] = 1;
    motor->message_num  = motor_send_num;
    motor->sender_group = motor_grouping;
}
```

旧版没有自动分组——应用层需要手动在 `Motor_Init_Config_s` 中正确设置所有 CAN 参数，包括 `sender_group`, `message_num` 和 `rx_id`，容易出错。

### 6.4 CAN 发送时机差异

```
旧版发送时机:
  DMMotorTask() {
      while(1) {
          计算 PID
          填 tx_buff
          CANTransmit()          ← 每电机每周期独立发送
          osDelay(1)
          发 DM_CMD_MOTOR_MODE   ← 每周期再发一条保活指令!
          osDelay(1)
      }
  }
  → 每电机每周期发 2 条 CAN 消息 (1条数据 + 1条保活)
  → 4电机 = 8条消息/周期, 8电机 = 16条/周期
  → CAN 总线负载高

新版发送时机:
  DMMotorControl() {
      for(电机) {
          switch(模式) {
              DJI_MODE:
                  填 sender_assignment[group].tx_buff[2*num]
                  DMMotorSetCommand(DM_CMD_MOTOR_MODE)  ← 保活在数据帧中
                  break;  ← 不独立发送
          }
      }
      for(0..5) {
          if(sender_enable_flag[i])
              CANTransmit(&sender_assignment[i])  ← 统一发送!
      }
  }
  → DJI_MODE 电机合并到6条分组消息中 + MIT/POSVEL/VEL 独立发送
  → 保活指令嵌入 DJI_MODE 数据帧
  → 大幅减少 CAN 总线占用
```

---

## 7. 编码器与角度计算对比

### 7.1 常量定义对比

| 常量 | 旧版 | 新版 | 变更 |
|------|------|------|------|
| 速度滤波系数 | `SPEED_SMOOTH_COEF` (0.85f) | `DM_SPEED_SMOOTH_COEF` (0.85f) | 加 `DM_` 前缀 |
| 电流滤波系数 | `CURRENT_SMOOTH_COEF` (0.9f) | `DM_CURRENT_SMOOTH_COEF` (0.9f) | 加 `DM_` 前缀 |
| 编码器→度 | `ECD_ANGLE_COEF_DM` (0.043945f) | `DM_ECD_ANGLE_COEF` (0.043945f) | 重命名 |
| 编码器→弧度 | `MOTOR_ECD_TO_RAD` | `DM_ECD_TO_RAD` | 重命名 |
| 编码器范围 | `ECD_RANGE` (8191) | `DM_ECD_RANGE` (8191) | 重命名 |
| 半圈范围 | `HALF_ECD_RANGE` (4096) | `DM_HALF_ECD_RANGE` (4096) | 重命名 |

> ⚠️ 所有常量从无前缀改为 `DM_` 前缀，避免与其他模块的宏名冲突。

### 7.2 CAN 解码函数对比

```c
// 旧版 DMMotorDecode (dmmotor.c:66-88)
static void DMMotorDecode(CANInstance *motor_can)
{
    // ...解析 ecd, speed_aps, real_current, temperature
    // 多圈计算 (与新版一致)
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

// 新版 DMMotorDecode (dm_motor.c:131-166)  
static void DMMotorDecode(CANInstance *_instance)
{
    // 1. 喂狗 DaemonReload
    // 2. 解析编码器 + 单圈角度
    // 3. 解析速度 + 低通滤波
    // 4. 解析电流 + 低通滤波
    // 5. 温度
    // 6. 多圈角度计算
    // 
    // 新增: motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
    //
    // 使用 #define DM_HALF_ECD_RANGE 替代硬编码 4096
}
```

核心逻辑一致，但新版：
1. 使用 `DM_HALF_ECD_RANGE` 宏替代硬编码数字
2. 参数命名 `_instance`（避免与全局变量混淆）
3. 新增 `DWT_GetDeltaT` 记录控制周期

---

## 8. 电机类型支持对比

### 8.1 硬件类型映射

| 硬件型号 | 旧版中的表示 | 新版中的表示 |
|----------|-------------|-------------|
| 达妙 M2006 替代 | (无专门类型) | `J4310` (Motor_Type_e) |
| 达妙 M3508 替代 | (无专门类型) | `J3507` (Motor_Type_e) |
| 达妙 GM6020 替代 | (无专门类型) | `J4340` (Motor_Type_e) |

### 8.2 旧版的类型问题

旧版在 `DMMotorInstance` 中**没有 `motor_type` 字段**。这意味着：

```c
// 旧版 gimbal.c 中的初始化
motor_yaw = DMMotorInit(&config, DJI_MODE);
// config.motor_type 被传入但 DMMotorInit 并未保存到实例中!
// → 只有 can_init_config 中保留了 tx_id/rx_id
// → 后续无法根据 motor_type 做差异化处理
```

新版修改：
```c
// 新版 DMMotorInit (dm_motor.c:277)
motor->motor_type = config->motor_type;  // ← 保存电机类型!
```

---

## 9. 初始化流程对比

### 9.1 DMMotorInit 对比

```
旧版 DMMotorInit (dmmotor.c:110-160):
═══════════════════════════════════════
1. malloc + memset
2. 保存 control_mode
3. 复制 motor_settings
4. 分别 Init gyro_PID, speed_PID, angle_PID ← 分散, 无 current_PID
5. 设置 other_angle_feedback_ptr, other_speed_feedback_ptr
6. tx_id 偏移 (MIT:不变, POSVEL:+0x100, VEL:+0x200, DJI:+0x3FE)
7. CANRegister (无自动分组)
8. DaemonRegister (reload_count=10)
9. DMMotorEnable + 发 DM_CMD_MOTOR_MODE
10. 加入 dm_motor_instance[idx++]
11. return motor

缺少:
- 不保存 motor_type
- 不初始化 current_PID
- 不初始化 speed_feedforward_ptr / current_feedforward_ptr
- 不做 DJI_MODE 自动分组


新版 DMMotorInit (dm_motor.c:270-343):
═══════════════════════════════════════
1. malloc + memset
2. 保存 motor_type + control_mode
3. 复制 motor_settings
4. 统一 Init current_PID, speed_PID, angle_PID ← 在 motor_controller 中
5. Init gyro_angle_PID ← 云台专用陀螺仪角度环
6. 设置 other_angle/speed_feedback_ptr
7. 设置 speed/current_feedforward_ptr    ← 新增前馈指针
8. tx_id 偏移 (DM_MIT_MODE:不变, DM_POSVEL:+0x100, DM_VEL:+0x200, DM_DJI:+0x3FE)
9. DJI_MODE → MotorSenderGrouping()     ← 新增自动分组
10. CANRegister
11. DaemonRegister (reload_count=10)
12. DMMotorEnable + DMMotorSetCommand(DM_CMD_MOTOR_MODE)
13. 加入 dm_motor_instance[idx++]
14. return motor
```

### 9.2 DMMotorControlInit 对比

```c
// ═══════════ 旧版 ═══════════
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    if (!idx) return;
    for (size_t i = 0; i < idx; i++) {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), 
                                            dm_motor_instance[i]);
    }
}
// 创建 N 个 FreeRTOS 任务，每个栈 128 字节

// ═══════════ 新版 ═══════════
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
        const char *mode_str[] = {"MIT", "POSVEL", "VEL", "DJI"};
        LOGINFO("[dm_motor] [%d] type=%d mode=%s can=%d tx_id=%d rx_id=%d",
                i, m->motor_type, mode_str[m->control_mode],
                can_bus, m->motor_can_instance->tx_id, m->motor_can_instance->rx_id);
    }
}
// 0 个任务创建, 仅输出注册日志便于调试
```

---

## 10. 核心控制回路对比

### 10.1 DJI_MODE 详细对比

```
═══════════════════════════════════════════════════════════════════
                    旧版 DJI_MODE (dmmotor.c:281-341)
═══════════════════════════════════════════════════════════════════

输入:
  maker_flag   (0=编码器, 1=陀螺仪)
  extern_flag  (0=yaw, 1=pitch)
  pid_ref[0]   (未使用)
  pid_ref[1]   (角度目标, 即 set2)
  pid_ref[2]   (未使用)

路径1: maker_flag=1, extern_flag=0 (陀螺仪-Yaw)
  pid_measure = measure->relative_angle_gyro
  → gyro_PID(pid_measure, pid_ref) → pid_gyro_out
  pid_measure = measure->gyro
  → speed_PID(pid_measure, pid_gyro_total + pid_gyro_out) → pid_out
  → set = (int16_t)pid_out                          ← 全局变量!

路径2: maker_flag=1, extern_flag=1 (陀螺仪-Pitch)
  pid_measure = measure->relative_angle_gyro
  → gyro_PID(pid_measure, pid_ref) → pid_gyro_out
  pid_measure = measure->gyro
  → speed_PID(pid_measure, pid_gyro_total + pid_gyro_out) → pid_out
  if (shoot_flag_dm == 1) { pid_out = pid_out; }    ← 无实际效果!
  → set_P = (int16_t)pid_out                        ← 全局变量!

路径3: maker_flag=0 (编码器)
  pid_measure = measure->relative_angle
  → angle_PID(pid_measure, pid_ref) → pid_out
  if (extern_flag == 0) → set = pid_out
  if (extern_flag == 1) → set_P = pid_out

终止: MOTOR_STOP → set=0, set_P=0
      LIMIT_MIN_MAX(set/set_P, DM_V_MIN, DM_V_MAX)

问题分析:
  1. set 和 set_P 是文件级全局变量 (static), 不是线程安全的
  2. set 和 set_P 赋值后... 不见了! 没有后续发送代码!
     djimode的发送逻辑不完整, set/set_P 没有被填入 CAN 缓冲区
  3. pid_gyro_total 始终为 0 (未赋值), pid_gyro_out+0 无意义
  4. shoot_flag_dm 的处理是空操作 (pid_out = pid_out)
  5. 没有分组发送, 每电机需要独自 CANTransmit
  6. 角度目标用的是 pid_ref (即 set2), 不是从外部传入的角度值


═══════════════════════════════════════════════════════════════════
                    新版 DJI_MODE (dm_motor.c:532-603)
═══════════════════════════════════════════════════════════════════

输入:
  motor_controller.pid_ref         (角度目标)
  motor_settings.angle_feedback_source (MOTOR_FEED / OTHER_FEED)
  motor_settings.speed_feedback_source (MOTOR_FEED / OTHER_FEED)
  motor_settings.outer_loop_type   (ANGLE_LOOP / SPEED_LOOP)
  motor_settings.close_loop_type   (哪些环参与控制)

--- 角度环 ---
if (outer_loop_type == ANGLE_LOOP) {
    if (angle_feedback_source == OTHER_FEED) {
        // 陀螺仪模式: gyro_angle_PID
        pid_ref = gyro_angle_PID(*other_angle_feedback_ptr, pid_ref);
    } else {
        // 编码器模式: angle_PID
        pid_ref = angle_PID(measure->total_angle, pid_ref);
    }
}

--- 速度环 ---
if (outer_loop_type & (ANGLE_LOOP|SPEED_LOOP)) {
    // 可选速度前馈
    if (SPEED_FEEDFORWARD) pid_ref += *speed_feedforward_ptr;
    
    if (speed_feedback_source == OTHER_FEED)
        pid_ref = speed_PID(*other_speed_feedback_ptr, pid_ref);
    else
        pid_ref = speed_PID(measure->speed_aps, pid_ref);
}

--- 电流环 ---
if (CURRENT_FEEDFORWARD) pid_ref += *current_feedforward_ptr;
if (close_loop_type & CURRENT_LOOP)
    pid_ref = current_PID(measure->real_current, pid_ref);

--- 输出 ---
if (feedback_reverse_flag) pid_ref *= -1;
if (stop_flag == MOTOR_STOP) pid_ref = 0;
set = (int16_t)pid_ref;

// 填入分组缓冲区
sender_assignment[group].tx_buff[2*num]   = set >> 8;
sender_assignment[group].tx_buff[2*num+1] = set & 0xFF;

优势:
  1. pid_ref 串级流转, 每个环的输出自动成为下一个环的输入
  2. 完整的角度→速度→电流三环串级结构
  3. 前馈注入点清晰 (速度前馈/电流前馈)
  4. 分组缓冲区统一填充, 循环结束后统一发送
  5. 无全局变量, 无 yaw/pitch 硬编码
```

### 10.2 MIT/POSVEL/VEL 模式对比

MIT/POSVEL/VEL 三种模式在发送端（MCU 侧不做闭环，直接发送目标值给电机驱动器）的逻辑上，两个版本**基本一致**。主要差异：

| 差异点 | 旧版 | 新版 |
|--------|------|------|
| 目标值来源 | `pid_ref[0]/[1]/[2]` 数组取 | `motor_controller.pid_ref` 标量 |
| POSVEL 速度源 | `pid_ref[1]` (应用层设置) | `*speed_feedforward_ptr` (前馈指针) |
| 反转处理 | 分散在各 case 中 | 统一在外层处理 |
| 空指针检查 | 无 | 各模式访问前馈指针前先判空 |

---

## 11. 守护进程与离线监控对比

### 11.1 离线回调

```c
// 旧版
static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor_ptr); // 重新使能
    DWT_Delay(0.1);
    // 无日志输出
}

// 新版
static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    uint16_t can_bus = (motor->motor_can_instance->can_handle == &hcan1) ? 1 : 2;
    LOGWARNING("[dm_motor] Motor lost, can_bus=%d, tx_id=%d",
               can_bus, motor->motor_can_instance->tx_id);
    // 有日志! 便于定位离线原因
    DMMotorSetCommand(motor, DM_CMD_MOTOR_MODE);
}
```

### 11.2 离线计数器

| 旧版 | 新版 |
|------|------|
| `lost_cnt` (uint32_t) — 定义了但**从未使用** | `feed_cnt` (uint32_t) — 在 DMMotorDecode 中通过 `DWT_GetDeltaT` 更新 |

---

## 12. 代码质量与健壮性对比

### 12.1 空指针保护

```c
// 旧版: 大部分函数无空指针检查
void DMMotorEnable(DMMotorInstance *motor) {
    motor->stop_flag = MOTOR_ENALBED;  // motor==NULL → HardFault!
}

// 新版: 每个公开 API 都有空指针检查
void DMMotorEnable(DMMotorInstance *motor) {
    if (motor == NULL) return;          // ← 安全!
    motor->stop_flag = MOTOR_ENALBED;
}
```

旧版中 `DMMotorSetRef` 有简单的空指针检查，但 `DMMotorEnable`, `DMMotorStop`, `DMMotorCaliEncoder`, `DMMotorOuterLoop`, `DMMotorShootFlag` 等函数都**没有**。

### 12.2 拼写错误修正

| 旧版拼写 | 新版修正 |
|----------|----------|
| `motor_can_instace` | `motor_can_instance` |
| `MOTOR_ENALBED` | `MOTOR_ENALBED` (继承自 motor_def.h) |
| `offest_angle` | `gyro_offset_angle` |

### 12.3 注释与文档

| 维度 | 旧版 | 新版 |
|------|------|------|
| 文件头描述 | 3行简短 TODO | 30行架构说明 + 流程图 |
| 函数注释 | 少量 Doxygen | 完整的 @brief/@param/@note/@attention |
| 分段注释 | 无 | `/* ==== 分段标题 ==== */` 清晰分隔 |
| 内联注释 | 少量 | 关键步骤有编号注释 (1. 2. 3. ...) |
| 头文件文档 | 无 | 35行重构说明 + 前置条件 + API 使用示例 |

### 12.4 死代码与逻辑错误

**旧版存在的问题:**

1. **DJI_MODE 发送不完整**: `set`/`set_P` 赋值后，没有代码将这些值填入 CAN 缓冲区并发送。`DJI_MODE` case 在 `if(extern_flag==0) break;` 后直接到 `default: while(1) LOGERROR(...)`——当 `extern_flag==0` (yaw) 时直接 break，当 `extern_flag==1` (pitch) 时落入 default 死循环！

2. **`pid_gyro_total` 未初始化**: 始终为 0，`pid_gyro_total + pid_gyro_out` 等价于 `pid_gyro_out`。

3. **发射前馈是空操作**: `if(motor->shoot_flag_dm==1) { pid_out=pid_out; }` 没有任何效果。

4. **`accel` 相关代码被注释掉**: pitch 轴的加速度反馈被注释，但保留了变量定义。

5. **`raw_gyro`, `relative_gyro_yaw`, `relative_gyro_pitch`**: 结构体中定义了但从未使用。

### 12.5 内存与性能

| 维度 | 旧版 | 新版 |
|------|------|------|
| 每实例内存 | ~400 bytes (含4个独立PID + measure含大量无用字段) | ~350 bytes (PID在motor_controller共用) |
| 栈内存 | 4×128 = 512 bytes (4电机) | 0 (共享MotorControlTask栈) |
| CAN 消息/周期(DJI_MODE) | N×2 (数据+保活) | 6(分组) + N(保活嵌入数据帧) |
| CPU占用 | 高 (任务切换) | 低 (顺序执行) |

---

## 13. 当前兼容性冲突分析

### 13.1 ⚠️ 严重警告：当前项目处于冲突状态

在 `motor_task.c` 中，**同时包含了两个版本的头文件**：

```c
// motor_task.c (第7-8行)
#include "dm_motor.h"   // 新版 v2.0
#include "dmmotor.h"    // 旧版 v1 beta ← 冲突!
```

Makefile 中**两个源文件都被编译**：

```makefile
# Makefile (第139-140行)
modules/motor/DMmotor/dmmotor.c \    # 旧版
modules/motor/DMmotor/dm_motor.c \   # 新版
```

### 13.2 符号冲突分析

两个版本定义了**完全相同名称的外部符号**：

| 符号 | 旧版 (dmmotor.o) | 新版 (dm_motor.o) | 冲突? |
|------|-----------------|-------------------|-------|
| `DMMotorInit` | ✓ | ✓ | ⚠️ 冲突 |
| `DMMotorSetRef` | ✓ (6参数) | ✓ (2参数) | ⚠️ 冲突+签名不同 |
| `DMMotorControl` | ✗ | ✓ | - |
| `DMMotorControlInit` | ✓ | ✓ | ⚠️ 冲突 |
| `DMMotorEnable` | ✓ | ✓ | ⚠️ 冲突 |
| `DMMotorStop` | ✓ | ✓ | ⚠️ 冲突 |
| `DMMotorCaliEncoder` | ✓ | ✓ | ⚠️ 冲突 |
| `DMMotorOuterLoop` | ✓ | ✓ | ⚠️ 冲突 |
| `DMMotorShootFlag` | ✓ | ✓ | ⚠️ 冲突 |
| `DMMotorSetCommand` | ✗ | ✓ | - |
| `DMMotorChangeFeed` | ✗ | ✓ | - |
| `DMMotorTask` | ✓ (非static) | ✗ | - |
| `dm_motor_instance` | static | static | 不冲突 (static) |
| `sender_assignment` | static | static | 不冲突 (static) |
| `DM_Motor_Measure_s` (类型) | ✓ (不同布局) | ✓ (不同布局) | ⚠️ 冲突 |
| `DMControl_Mode_e` (枚举) | ✓ (值不同) | ✓ (值不同) | ⚠️ 冲突 |
| `DMMotorInstance` (类型) | ✓ (不同布局) | ✓ (不同布局) | ⚠️ 冲突 |

### 13.3 当前状态的实际行为

由于链接器在遇到重复符号时的行为**取决于工具链和链接顺序**：

- **最可能**: 链接器取第一个找到的符号（取决于 Makefile 中 .o 文件的链接顺序）
- **可能导致**: 部分函数调用旧版、部分调用新版，行为完全不可预测
- **类型冲突**: 两个 `DM_Motor_Measure_s` 和 `DMMotorInstance` 结构体布局不同，如果应用层用旧版结构体布局访问新版实例（或反过来），会导致**内存越界读写**

### 13.4 编译可能通过但运行时危险

由于两个版本的函数签名不同：
- `DMMotorSetRef(motor, ref1, ref2, ref3, flag1, flag2)` — 6参数
- `DMMotorSetRef(motor, ref)` — 2参数

如果 gimbal.c 调用的是6参数版本但链接到新版2参数版本，会导致**栈溢出或参数错位**。

---

## 14. 迁移指南

### 14.1 立即需要做的（修复编译冲突）

```
步骤1: 从 Makefile 中移除旧版源文件
  # 删除或注释这一行:
  modules/motor/DMmotor/dmmotor.c \

步骤2: 从 motor_task.c 中移除旧版头文件
  # 删除或注释这一行:
  #include "dmmotor.h"

步骤3: 检查是否还有其他文件 include "dmmotor.h"
  grep -r '"dmmotor.h"' application/ modules/

步骤4: clean build 并测试
  make clean && make
```

### 14.2 应用层代码迁移

#### gimbal.c 迁移

```c
// ═══════════ 旧版代码 (当前) ═══════════

// 初始化
motor_yaw = DMMotorInit(&yaw_config, DJI_MODE);
motor_pitch = DMMotorInit(&pitch_config, DJI_MODE);

// 设目标值 (6参数)
DMMotorSetRef(motor_yaw, target_angle, 0, 0, 0, 0);   // 编码器-yaw
DMMotorSetRef(motor_pitch, target_angle, 0, 0, 1, 1);  // 陀螺仪-pitch

// 初始化控制
DMMotorControlInit();  // 创建 FreeRTOS 任务


// ═══════════ 新版代码 (迁移后) ═══════════

// 初始化 - 接口相同，但枚举值加 DM_ 前缀
yaw_config.motor_type = J4310;    // 新增: 需要设置电机硬件类型
motor_yaw = DMMotorInit(&yaw_config, DM_DJI_MODE);

pitch_config.motor_type = J3507;
motor_pitch = DMMotorInit(&pitch_config, DM_DJI_MODE);

// 配置反馈源 (替代 maker_flag/extern_flag)
// Yaw: 编码器模式
DMMotorOuterLoop(motor_yaw, ANGLE_LOOP);
DMMotorChangeFeed(motor_yaw, ANGLE_LOOP, MOTOR_FEED);

// Pitch: 陀螺仪模式
DMMotorOuterLoop(motor_pitch, ANGLE_LOOP);
DMMotorChangeFeed(motor_pitch, ANGLE_LOOP, OTHER_FEED);
DMMotorChangeFeed(motor_pitch, SPEED_LOOP, OTHER_FEED);
motor_pitch->motor_controller.other_angle_feedback_ptr = &imu_data.Pitch;
motor_pitch->motor_controller.other_speed_feedback_ptr = &imu_data.Gyro[1];

// 设目标值 (2参数! 与 DJIMotorSetRef 一致)
DMMotorSetRef(motor_yaw, target_angle);
DMMotorSetRef(motor_pitch, target_angle);

// 初始化控制 (不再创建任务!)
DMMotorControlInit();  // 仅打印日志

// 在 MotorControlTask 中调用 (已在 motor_task.c 中)
// DMMotorControl();  ← 与 DJIMotorControl() 在同一个 1kHz 任务中
```

#### upper.c 迁移

```c
// 旧版
yaw3_motor = DMMotorInit(&yaw3_config, POSVEL_MODE);
DMMotorSetRef(yaw3_motor, target_pos, target_vel, 0, 0, 0);

// 新版
yaw3_config.motor_type = J4310;
yaw3_motor = DMMotorInit(&yaw3_config, DM_POSVEL_MODE);
DMMotorSetRef(yaw3_motor, target_pos);
// 速度目标通过 speed_feedforward_ptr 注入
```

### 14.3 枚举值迁移速查

| 旧版 | 新版 |
|------|------|
| `MIT_MODE` | `DM_MIT_MODE` |
| `POSVEL_MODE` | `DM_POSVEL_MODE` |
| `VEL_MODE` | `DM_VEL_MODE` |
| `DJI_MODE` | `DM_DJI_MODE` |
| `DM_CMD_MOTOR_MODE` | `DM_CMD_MOTOR_MODE` (不变) |
| `DM_CMD_RESET_MODE` | `DM_CMD_RESET_MODE` (不变) |
| `DM_CMD_ZERO_POSITION` | `DM_CMD_ZERO_POSITION` (不变) |
| `DM_CMD_CLEAR_ERROR` | `DM_CMD_CLEAR_ERROR` (不变) |
| `DMMotor_Mode_e` | `DMMotor_Command_e` |

### 14.4 常量迁移速查

| 旧版 | 新版 |
|------|------|
| `SPEED_SMOOTH_COEF` | `DM_SPEED_SMOOTH_COEF` |
| `CURRENT_SMOOTH_COEF` | `DM_CURRENT_SMOOTH_COEF` |
| `ECD_ANGLE_COEF_DM` | `DM_ECD_ANGLE_COEF` |
| `MOTOR_ECD_TO_RAD` | `DM_ECD_TO_RAD` |
| `ECD_RANGE` | `DM_ECD_RANGE` |
| `HALF_ECD_RANGE` | `DM_HALF_ECD_RANGE` |
| `DM_P_MIN / MAX` | `DM_P_MIN / MAX` (不变) |
| `DM_V_MIN / MAX` | `DM_V_MIN / MAX` (不变) |
| `DM_T_MIN / MAX` | `DM_T_MIN / MAX` (不变) |

### 14.5 字段访问迁移

```c
// 旧版访问测量数据
motor->measure.gyro          // 角速度
motor->measure.accel         // 加速度
motor->measure.offest_angle  // 零点偏移

// 新版访问测量数据
motor->measure.gyro_rate          // 角速度 (重命名)
motor->measure.gyro_accel         // 加速度 (重命名)
motor->measure.gyro_offset_angle  // 零点偏移 (重命名+拼写修正)

// 旧版访问 PID
motor->angle_PID
motor->speed_PID
motor->gyro_PID

// 新版访问 PID
motor->motor_controller.angle_PID
motor->motor_controller.speed_PID
motor->motor_controller.current_PID
motor->gyro_angle_PID   // 陀螺仪角度环, 仍在实例层

// 旧版访问目标值
motor->pid_ref[0]  // 位置
motor->pid_ref[1]  // 速度
motor->pid_ref[2]  // 电流

// 新版访问目标值
motor->motor_controller.pid_ref  // 单一标量, 串级流转
```

---

## 15. 总结表

### 15.1 综合评分

| 维度 | 旧版 (dmmotor) | 新版 (dm_motor) |
|------|:---:|:---:|
| 架构合理性 | ★★☆☆☆ | ★★★★★ |
| 与 DJI 驱动一致性 | ★☆☆☆☆ | ★★★★★ |
| 代码可读性 | ★★☆☆☆ | ★★★★★ |
| API 易用性 | ★★☆☆☆ | ★★★★★ |
| RTOS 资源效率 | ★★☆☆☆ | ★★★★★ |
| CAN 总线效率 | ★★☆☆☆ | ★★★★☆ |
| 健壮性 (空指针保护) | ★☆☆☆☆ | ★★★★★ |
| 可扩展性 (8电机) | ★★☆☆☆ | ★★★★★ |
| 注释文档 | ★☆☆☆☆ | ★★★★★ |
| 功能完整性 (电流环/前馈) | ★★☆☆☆ | ★★★★★ |

### 15.2 一句话总结

**旧版 (dmmotor)** 是一个为每颗达妙电机创建独立 FreeRTOS 任务的原型驱动，API 复杂（`DMMotorSetRef` 需要6个参数）、DJI_MODE 下的发送逻辑不完整，且与项目中 DJI 电机的集中式控制架构完全不一致。

**新版 (dm_motor)** 是一个全面对齐 DJI 电机驱动风格的正式版本——采用集中式 `DMMotorControl()` 函数在 `MotorControlTask` 中以 1kHz 统一执行，支持完整的串级 PID（角度→速度→电流）+ 前馈 + 陀螺仪反馈源切换，API 简洁（`DMMotorSetRef` 只需2个参数），并具备自动 CAN 分组、ID 冲突检测、空指针保护和详细日志输出。

### 15.3 建议

1. **立即**: 从 Makefile 中移除 `dmmotor.c`，从 `motor_task.c` 中移除 `#include "dmmotor.h"`，解决符号冲突
2. **短期**: 将 `gimbal.c` 和 `upper.c` 迁移到新版 API
3. **长期**: 删除 `dmmotor.c` 和 `dmmotor.h` 文件，避免混淆
4. **注意**: 新版需要在 `Motor_Type_e` 枚举中使用 `J4310`/`J3507`/`J4340` 来标识达妙电机硬件型号，与旧版复用 `M2006`/`M3508` 的做法不同

---

> **文档结束** — 如需进一步的技术细节，请阅读源文件:
> - `modules/motor/DMmotor/dmmotor.c` + `dmmotor.h` (旧版 v1 beta)
> - `modules/motor/DMmotor/dm_motor.c` + `dm_motor.h` (新版 v2.0)
> - `modules/motor/motor_def.h` (公用数据结构)
> - `modules/motor/motor_task.c` (控制任务入口)
