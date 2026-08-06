# dm_motor v2.0 内部缺陷分析

## （仅审计新版代码本身，不涉及与 gimbal.c/upper.c 的兼容适配）

---

> **审计日期**: 2026-08-06
> **审计文件**: `modules/motor/DMmotor/dm_motor.c` (649行) + `dm_motor.h` (262行)
> **参照基准**: 旧版 `dmmotor.c`、DJI 驱动 `dji_motor.c`、`motor_def.h`、`controller.h`、`bsp_can.h`
> **结论概要**: 发现 **2 个致命缺陷**，**3 个高危缺陷**，**4 个中危缺陷**，**4 个低危缺陷**

---

## 目录

1. [🔴 CRITICAL-1: DJI_MODE 输出无限制，int16_t 溢出可导致电机反向](#c1)
2. [🔴 CRITICAL-2: 与 DJI 驱动 CAN ID 冲突，电机收到错误控制数据](#c2)
3. [🟠 HIGH-1: DMMotorInit 数组越界无保护](#h1)
4. [🟠 HIGH-2: DJI_MODE 速度环空指针回退为 0 导致飞车](#h2)
5. [🟠 HIGH-3: POSVEL_MODE 速度目标静默断裂](#h3)
6. [🟡 MEDIUM-1: DJI_MODE 每周期冗余 CAN 发送导致总线负载过高](#m1)
7. [🟡 MEDIUM-2: MIT 模式 STOP 行为语义不明确](#m2)
8. [🟡 MEDIUM-3: MotorSenderGrouping 对未知类型死循环](#m3)
9. [🟡 MEDIUM-4: 编码器多圈计算无溢出保护](#m4)
10. [🟢 LOW-1: MIT 模式 Kp/Kd 硬编码为零](#l1)
11. [🟢 LOW-2: sender_assignment 缓冲区未在周期间清理](#l2)
12. [🟢 LOW-3: DMMotorSetCommand 与 DMMotorControl 共用 tx_buff 无锁](#l3)
13. [🟢 LOW-4: 电流环零参数陷阱](#l4)

---

<a name="c1"></a>

## 🔴 CRITICAL-1: DJI_MODE 输出无限制，int16_t 溢出可导致电机反向

### 位置

`dm_motor.c:592`

```c
set = (int16_t)pid_ref;
```

### 问题

`pid_ref` 经过**角度环→速度环→电流环**三级串级 PID，每一级的 `MaxOut` 可分别配置。三级输出累加后，`pid_ref` 可能远超 `int16_t` 范围（-32768 ~ 32767）。

**触发路径:**

```
假设:
  角度环 angle_PID.MaxOut = 15000     (典型值)
  速度环 speed_PID.MaxOut = 2000      (典型值)
  电流环 current_PID.MaxOut = 20000   (典型值)
  速度前馈 = 500

串级累积:
  pid_ref = DMMotorSetRef() 传入        →   0
  pid_ref = angle_PID(...)             → 15000  (饱和)
  pid_ref += speed_feedforward (500)   → 15500
  pid_ref = speed_PID(..., 15500)      →  2000  (速度环饱和)
  pid_ref += current_feedforward (300) →  2300
  pid_ref = current_PID(..., 2300)     → 20000  (电流环饱和)
  
  set = (int16_t)20000 → OK

但如果 MaxOut 设大 (调试时):
  pid_ref = current_PID(..., 50000)    → 35000
  
  set = (int16_t)35000 → 二进制补码溢出!
  35000 = 0x88B8 → 解释为 int16_t = -30536
  → 电机命令从正向最大突然变为反向大电流
```

### 对比

**旧版 dmmotor.c:333-334** 有明确的限幅：

```c
LIMIT_MIN_MAX(set, DM_V_MIN, DM_V_MAX);     // ±60 rad/s
LIMIT_MIN_MAX(set_P, DM_V_MIN, DM_V_MAX);
```

**DJI 驱动 dji_motor.c:486** 同样没有 `LIMIT_MIN_MAX`：

```c
set = (int16_t)pid_ref;
```

所以这严格来说不是 DM v2.0 独创的 bug——它继承自 DJI 驱动。但 DJI 驱动的 MaxOut 经过长期调优通常不会超限，而 DM 驱动的 PID 参数是新的、未经验证的，风险更高。

### 修复

```c
// 在 set = (int16_t)pid_ref; 之前加一行:
LIMIT_MIN_MAX(pid_ref, -30000.0f, 30000.0f);  // 安全限幅，防止 int16 溢出
set = (int16_t)pid_ref;
```

> `±30000` 足够覆盖 DM 电机的最大电流指令，同时避开 ±32768 的溢出边界。

---

<a name="c2"></a>

## 🔴 CRITICAL-2: 与 DJI 驱动 CAN ID 冲突，电机收到错误控制数据

### 位置

`dm_motor.c:57-70` (sender_assignment 定义)

### 问题

DM 驱动的 `sender_assignment` 分组 CAN ID 与 DJI 驱动的分组 CAN ID **部分重叠**：

```
             DJI (dji_motor.c)           DM (dm_motor.c)
             ════════════════            ═══════════════
CAN1 组0:    0x1FF                       0x3FE           ← 不冲突 ✓
CAN1 组1:    0x200                       0x200           ← ⚠️ 完全相同!
CAN1 组2:    0x2FF                       0x2FF           ← ⚠️ 完全相同!
CAN2 组0:    0x1FF                       0x3FE           ← 不冲突 ✓
CAN2 组1:    0x200                       0x200           ← ⚠️ 完全相同!
CAN2 组2:    0x2FF                       0x2FF           ← ⚠️ 完全相同!
```

### 后果

`MotorControlTask()` 的调用顺序：

```c
void MotorControlTask() {
    DJIMotorControl();   // 发送 6 组 DJI CAN 帧
    DMMotorControl();    // 发送 6 组 DM CAN 帧 + N 条保活指令
}
```

**同一 CAN 总线上出现两条 ID 完全相同的 CAN 帧**（一条来自 DJI 驱动、一条来自 DM 驱动），分别承载不同电机的控制数据。

DM 电机（如 J4310，tx_id=1）收到的数据：
- 时刻 T: DJI 的 0x200 帧 → bytes[0-1] 是 DJI M3508 电机1 的电流指令
- 时刻 T+Δt: DM 的 0x200 帧 → bytes[0-1] 是 DM J4310 电机1 的电流指令

**DM 电机会在两个帧之间交替接收 DJI 电机的指令和 DM 电机的指令**，导致电机力矩抖动或失控。

### 触发条件

1. DJI 电机和 DM 电机**共用同一 CAN 总线**（例如 CAN1 上既有 DJI M3508 又有 DM J4310）
2. 两者的 tx_id 分别落在使能了 sender_enable_flag 的组号内

### 注意

旧版 dmmotor 的 DJI_MODE 中，`sender_assignment` 虽然定义了同样的 ID，但**控制发送逻辑不完整**（`set`/`set_P` 赋值后从未写入 CAN 缓冲区，见旧版代码分析），实际从未通过 sender_assignment 发送数据。所以这个冲突是 v2.0 **修复了发送逻辑后才被新引入的**。

### 修复

**方案A（推荐）:** 将 DM 的 0x200 和 0x2FF 改为 DM 专用 ID

```c
// dm_motor.c sender_assignment 修改:
[1] = {..., .txconf.StdId = 0x300, ...},  // 替代 0x200
[2] = {..., .txconf.StdId = 0x2FE, ...},  // 替代 0x2FF
[4] = {..., .txconf.StdId = 0x300, ...},  // CAN2 同理
[5] = {..., .txconf.StdId = 0x2FE, ...},
```

但需要确认**达妙电机固件是否支持在这些 ID 上接收 DJI 兼容控制帧**。如果达妙电机固件硬编码只监听 0x200/0x1FF/0x2FF，则此方案不可行。

**方案B:** 如果达妙电机必须用 0x200/0x2FF，则需要**合并 DJI 和 DM 的 sender_assignment**

将 DM 电机的分组发送整合到 DJI 的 sender_assignment 中，确保同一个 CAN ID 的帧中同时包含 DJI 和 DM 电机的数据（通过 tx_id 在 bytes 中的不同位置区分）。

**方案C:** 强制 CAN 总线隔离——DJI 电机全在 CAN1，DM 电机全在 CAN2（或反过来）。这是最简单但最不灵活的方案。

---

<a name="h1"></a>

## 🟠 HIGH-1: DMMotorInit 数组越界无保护

### 位置

`dm_motor.c:341`

```c
dm_motor_instance[idx++] = motor;
```

### 问题

`dm_motor_instance` 的容量是 `DM_MOTOR_CNT = 8`（定义在 `dm_motor.h:46`）。如果注册了第 9 颗电机，`dm_motor_instance[8]` 会越界写入，破坏栈上或 `.bss` 段中紧随其后的数据。

旧版同样有此问题（上限 `DM_MOTOR_CNT = 4`，越界概率更高），但新版将上限从 4 提高到 8 降低了触发概率，却**没有从根本上修复**。

### 修复

```c
if (idx >= DM_MOTOR_CNT) {
    LOGERROR("[dm_motor] Too many motors registered! Max: %d", DM_MOTOR_CNT);
    free(motor);
    return NULL;
}
dm_motor_instance[idx++] = motor;
```

---

<a name="h2"></a>

## 🟠 HIGH-2: DJI_MODE 速度环空指针回退为 0 导致飞车

### 位置

`dm_motor.c:562-566`

```c
if (setting->speed_feedback_source == OTHER_FEED) {
    /* 陀螺仪模式: 使用 IMU 角速度 */
    pid_measure = (ctrl->other_speed_feedback_ptr)
                ? *ctrl->other_speed_feedback_ptr
                : measure->gyro_rate;        // ← 回退值
}
```

### 问题

当 `speed_feedback_source = OTHER_FEED` 但 `other_speed_feedback_ptr` 为 NULL 时，回退使用 `measure->gyro_rate`。然而：

1. `gyro_rate` **不在 CAN 解码中被填充**——`DMMotorDecode()` 只解析编码器位置、转速(RPM)、电流和温度，不写 `gyro_rate`
2. `memset(motor, 0, sizeof(...))` 时 `gyro_rate` 被初始化为 **0.0f**
3. `gyro_rate` 也不在 `dm_motor.h` 的任何注释中被说明为"需要应用层填充"

**该字段在整个 dm_motor 驱动的正常数据流中始终为 0。**

如果用户设置 `OTHER_FEED` 但忘记设指针：
- 速度环反馈 = 0（实际电机可能在高速旋转）
- 速度环设定点 = 角度环输出（非零）
- 误差巨大 → 速度环输出饱和 → **电机猛烈加速**

### DJI 驱动的等价代码

```c
// dji_motor.c:460-463
if (motor_setting->speed_feedback_source == OTHER_FEED)
    pid_measure = *motor_controller->other_speed_feedback_ptr;  // ← 直接解引用！
else
    pid_measure = measure->speed_aps;
```

DJI 驱动直接解引用 `other_speed_feedback_ptr`，没有 NULL 检查——如果指针为 NULL 会直接 HardFault。

DM v2.0 的"防御性"回退逻辑反而更危险：**避免了 HardFault，但换来了静默的飞车**。

### 修复

```c
if (setting->speed_feedback_source == OTHER_FEED) {
    if (ctrl->other_speed_feedback_ptr == NULL) {
        LOGERROR("[dm_motor] OTHER_FEED set but other_speed_feedback_ptr is NULL!");
        motor->stop_flag = MOTOR_STOP;
        break;
    }
    pid_measure = *ctrl->other_speed_feedback_ptr;
}
```

角度环同理 (`dm_motor.c:540-542`):

```c
pid_measure = (ctrl->other_angle_feedback_ptr)
            ? *ctrl->other_angle_feedback_ptr
            : measure->relative_angle_gyro;  // ← 同样可能为 0
```

角度环回退到 `relative_angle_gyro`，虽然不会像速度环那样直接飞车，但会产生错误的控制行为。建议同样改为：指针为空时报告错误并停止。

---

<a name="h3"></a>

## 🟠 HIGH-3: POSVEL_MODE 速度目标静默断裂

### 位置

`dm_motor.c:497-514`

```c
case DM_POSVEL_MODE: {
    float pos = pid_ref;
    float vel = 0;                                 // ← 初始化为 0!
    if (ctrl->speed_feedforward_ptr)
        vel = *ctrl->speed_feedforward_ptr;        // ← 只能通过前馈指针获取速度
    ...
    memcpy(can->tx_buff,     &pos, 4);
    memcpy(can->tx_buff + 4, &vel, 4);             // ← vel 发送
    CANTransmit(can, 1);
    break;
}
```

### 问题

POSVEL_MODE 需要同时发送**位置目标**和**速度目标**给电机驱动器。旧版从 `pid_ref[0]` 和 `pid_ref[1]` 分别获取位置和速度；新版 `DMMotorSetRef()` 只传一个参数，速度目标**只能通过 `speed_feedforward_ptr` 指针获取**。

如果用户没有设置 `speed_feedforward_ptr`：
- `vel` 保持为 **0**
- 电机驱动收到位置=目标位置，速度=0
- 电机以位置模式运行，无速度前馈 → 响应迟缓、跟随精度下降
- **不会报任何编译警告或运行时错误**——静默退化

### 修复

提供一个回退机制，当 `speed_feedforward_ptr` 为空时用 0 作为默认速度并至少打一条 LOGWARNING。或者在 `DMMotorInit` 中检测 POSVEL_MODE 但没有设置速度前馈指针时发出警告。

---

<a name="m1"></a>

## 🟡 MEDIUM-1: DJI_MODE 每周期冗余 CAN 发送导致总线负载过高

### 位置

`dm_motor.c:601`

```c
/* 保持电机使能 (每周期发送命令, 防止自动失能) */
DMMotorSetCommand(motor, DM_CMD_MOTOR_MODE);
```

### 问题

`DMMotorControl()` 以 1kHz 运行。DJI_MODE 下，**每颗 DM 电机每个周期**都调用一次 `DMMotorSetCommand`，每次执行一条 `CANTransmit`。

```c
void DMMotorSetCommand(DMMotorInstance *motor, DMMotor_Command_e cmd)
{
    memset(motor->motor_can_instance->tx_buff, 0xff, 7);
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instance, 1);  // ← 每次一条 CAN 帧
}
```

**CAN 总线负载计算（假设 4 颗 DJI_MODE 电机，1kHz）：**

```
DJIMotorControl():    6 条分组帧 × 1000 =  6000 帧/秒
DMMotorControl():     6 条分组帧 × 1000 =  6000 帧/秒
DM 保活 DMMotorSetCommand(): 4 × 1000  =  4000 帧/秒  ← 仅 DM 电机
                                        ─────────
                                        16000 帧/秒

CAN 1Mbps 理论最大帧率: ~8000 帧/秒 (每帧 ~130μs)
16000 > 8000 → 超过 CAN 总线理论承载能力
```

STM32F407 的 CAN 外设有 3 个发送邮箱。当邮箱全满时，`CANTransmit` 内部调用 `HAL_CAN_AddTxMessage` 会等待最多 `timeout` 毫秒。由于 DM 的保活指令使用 `timeout=1`（1ms），如果邮箱已满，会发生 CAN 发送错误。

### 对比旧版

旧版 `DMMotorTask` 的保活频率约 500Hz（`osDelay(1)+osDelay(1)` = 每周期 2ms），但同样每条保活指令独立发送。

### 修复

**不要每周期都发送保活指令。** 达妙电机的自动失能超时一般在 100-500ms 量级。

```c
// 改为每 100ms 发送一次保活
static uint8_t keepalive_counter = 0;
if (++keepalive_counter >= 100) {  // 每 100 个周期 (100ms)
    keepalive_counter = 0;
    DMMotorSetCommand(motor, DM_CMD_MOTOR_MODE);
}
```

或者将保活指令嵌入分组帧（与 DJI 驱动对 DJI 电机的处理一致——DJI 驱动没有保活指令，因为 DJI 电机只根据控制数据超时判断离线）。

---

<a name="m2"></a>

## 🟡 MEDIUM-2: MIT 模式 STOP 行为语义不明确

### 位置

`dm_motor.c:478-479`

```c
if (motor->stop_flag == MOTOR_STOP)
    mit.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
```

### 问题

STOP 时 MIT 模式仍然发送完整的 MIT 控制帧（position=0, velocity=0, Kp=0, Kd=0, torque=0），而不是发送 `DM_CMD_RESET_MODE` 让电机进入停止状态。

达妙电机收到 Kp=0, Kd=0, torque=0 的 MIT 帧后的行为取决于固件实现：
- 可能自由旋转（期望行为）
- 可能保持最后一帧非零指令的状态
- 可能进入某种错误状态

这与直接发送 `DM_CMD_RESET_MODE`（明确停止电机）的语义有本质区别。

### 修复

```c
if (motor->stop_flag == MOTOR_STOP) {
    DMMotorSetCommand(motor, DM_CMD_RESET_MODE);  // 明确停止
    break;  // 不再发送控制帧
}
```

---

<a name="m3"></a>

## 🟡 MEDIUM-3: MotorSenderGrouping 对未知类型死循环

### 位置

`dm_motor.c:232-235`

```c
default:
    while (1)
        LOGERROR("[dm_motor] Unsupported motor type for grouping!");
    return;
```

### 问题

如果 `motor_type` 不是 `J4310`、`J3507` 或 `J4340`（例如错误地传入了 `M2006` 或 `M3508`），系统会进入**死循环**，所有电机控制停止。

这是从 DJI 驱动 (`dji_motor.c:113-114`) 克隆的代码风格，但在 DM 驱动中风险更高——因为 DM 驱动的 `motor_type` 字段是新增的，用户可能用错。

### 修复

```c
default:
    LOGERROR("[dm_motor] Unsupported motor type %d for grouping!", motor->motor_type);
    return;  // 返回但不死循环，让调用者处理
```

同时 `DMMotorInit` 需要检查 `MotorSenderGrouping` 的返回值或状态。

---

<a name="m4"></a>

## 🟡 MEDIUM-4: 编码器多圈计算无溢出保护

### 位置

`dm_motor.c:160-165`

```c
/* 6. 多圈角度计算 (假设两次采样间转角 < 180°) */
if (m->ecd - m->last_ecd > DM_HALF_ECD_RANGE)
    m->total_round--;
else if (m->ecd - m->last_ecd < -DM_HALF_ECD_RANGE)
    m->total_round++;
```

### 问题

注释已经说明："**假设两次采样间转角 < 180°**"。在 1kHz 采样率下，180° 对应 10800°/s = 30 rps = 1800 rpm。大部分电机运行在这个范围内。

但如果电机转速超过 ~1800 rpm（例如 M3508 最大转速约 9000 rpm），两次采样间可能转过 >180°，多圈计算会出错。DM 电机的速度上限 `DM_V_MAX = 60 rad/s ≈ 573 rpm`，所以正常使用时不会触发。

**真正的风险**：如果 CAN 通信中断（电机离线），`DMMotorDecode` 不会被调用，`total_round` 保持旧值。当通信恢复且电机在离线期间转了 N 圈，多圈角度会永久偏移 N×360°。

DJI 驱动有同样的假设，但 DJI 电机的 daemon reload_count=2（20ms 超时），而 DM 的 reload_count=10（100ms 超时）。在 100ms 的空白期内，电机可以转过很多圈。

---

<a name="l1"></a>

## 🟢 LOW-1: MIT 模式 Kp/Kd 硬编码为零

### 位置

`dm_motor.c:475-476`

```c
mit.Kp = 0;
mit.Kd = 0;
```

### 问题

MIT 模式是一种**阻抗控制**协议：`τ = Kp×(θ_des - θ) + Kd×(ω_des - ω) + τ_ff`。Kp 和 Kd 分别代表驱动侧的**虚拟刚度**和**虚拟阻尼**。

当前实现将 Kp 和 Kd 硬编码为 0，意味着 MIT 模式退化为**纯前馈力矩控制**——驱动侧不提供任何位置/速度闭环。这在某些场景（如高带宽力控）是合理的，但对其他场景（如需要驱动侧提供基本稳定的直驱关节）则不够。

与旧版一致，所以这不算新引入的问题，但 v2.0 作为"完整版"，应该提供可配置的 Kp/Kd。

### 修复

建议在 `DMMotorInstance` 或 `Motor_Control_Setting_s` 中增加 `mit_Kp` 和 `mit_Kd` 字段。

---

<a name="l2"></a>

## 🟢 LOW-2: sender_assignment 缓冲区未在周期间清理

### 位置

`dm_motor.c:597-598`

```c
sender_assignment[group].tx_buff[2 * num]     = (uint8_t)(set >> 8);
sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff);
```

### 问题

`sender_assignment[group].tx_buff` 是静态全局数组，不会被 memset 清零。如果某组之前有 4 颗电机、后来只有 2 颗（可能是因为其他电机后来被关闭），则未更新的字节（bytes 4-7）保留旧值——发出包含两个已不存在电机的过时数据的 CAN 帧。

### 缓解因素

DJI 驱动（dji_motor.c:500-501）有完全相同的设计：

```c
if (motor->stop_flag == MOTOR_STOP)
    memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
```

DM 驱动通过 `pid_ref = 0` → `set = 0` 实现了等效的清零。只要 STOP 的电机仍然在循环中迭代（它们确实在），其对应字节就会被写为 0。所以这不是真正的 bug——只要 STOP 的电机没有被从 `dm_motor_instance[]` 数组中移除。

---

<a name="l3"></a>

## 🟢 LOW-3: DMMotorSetCommand 与 DMMotorControl 共用 tx_buff 无锁

### 位置

`dm_motor.c:108-116` 和 `dm_motor.c:597-598`

```c
// DMMotorSetCommand (可能来自应用层 RobotTask 200Hz):
memset(motor->motor_can_instance->tx_buff, 0xff, 7);
motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd;
CANTransmit(motor->motor_can_instance, 1);

// DMMotorControl DJI_MODE (来自 MotorControlTask 1kHz):
sender_assignment[group].tx_buff[2 * num] = ...;
// ...
CANTransmit(&sender_assignment[group], 1);
```

### 问题

两种 tx_buff 是不同的缓冲区：
- `motor->motor_can_instance->tx_buff`（DMMotorSetCommand 使用）
- `sender_assignment[group].tx_buff`（DMMotorControl 使用）

所以**不会在 tx_buff 层面冲突**。但是两个 `CANTransmit` 都操作同一个 CAN 外设（hcan1/hcan2），共享 3 个发送邮箱。如果应用层在 RobotTask 中调用了 `DMMotorCaliEncoder()` → `DMMotorSetCommand(DM_CMD_ZERO_POSITION)`，而此时 MotorControlTask 正在发送分组数据，**邮箱竞争**可能导致超时。

### 影响

`CANTransmit` 的超时参数为 1ms。如果 3 个邮箱恰好全满，校零指令会等待 1ms 后失败返回。电机校零失败但不会崩溃。

### 修复

可以将 `DMMotorCalibrateEncoder` 的超时增大到 10ms，或在注释中告知用户"校零期间勿做其他操作"。

---

<a name="l4"></a>

## 🟢 LOW-4: 电流环零参数陷阱

### 位置

`dm_motor.c:579-582`

```c
if (setting->close_loop_type & CURRENT_LOOP) {
    pid_ref = PIDCalculate(&ctrl->current_PID, measure->real_current, pid_ref);
}
```

### 问题

如果用户配置了 `close_loop_type |= CURRENT_LOOP`，但在 `Motor_Controller_Init_s` 中没有初始化 `current_PID`（所有字段保持为 `memset` 后的 0），则：
- Kp=0, Ki=0, Kd=0 → PID 输出恒为 0
- 电机完全不输出力矩

`PID_Init_Config_s` 的默认 Kp/Ki/Kd 都是 0，因为 `memset(0)`。

### 影响

仅当用户显式启用了 `CURRENT_LOOP` 但忘记配置 `current_PID` 参数时触发。由于 gimbal.c 的配置中 `close_loop_type = ANGLE_LOOP|SPEED_LOOP`（无 CURRENT_LOOP），当前不受影响。

DJI 驱动有完全相同的设计，这是 `Motor_Controller_s` 框架的通用问题，非 DM 独有。

### 修复

在 `DMMotorInit` 中检测：

```c
if ((motor->motor_settings.close_loop_type & CURRENT_LOOP)
    && motor->motor_controller.current_PID.Kp == 0.0f
    && motor->motor_controller.current_PID.MaxOut == 0.0f) {
    LOGWARNING("[dm_motor] CURRENT_LOOP enabled but current_PID not configured!");
}
```

---

## 总结

### 按严重度排序的修复清单

| 优先级 | ID | 缺陷 | 修复难度 | 建议 |
|--------|----|------|---------|------|
| **P0** | C1 | DJI_MODE 输出无 int16 限幅 | 一行代码 | `LIMIT_MIN_MAX(pid_ref, -30000, 30000)` |
| **P0** | C2 | CAN ID 与 DJI 冲突 | 架构级 | 确认 DM 电机 CAN ID 规范后选择方案 |
| **P1** | H1 | 数组越界无保护 | 三行代码 | 加 `idx >= DM_MOTOR_CNT` 检查 |
| **P1** | H2 | 空指针回退飞车 | 四行代码 | 改为报错+停机 |
| **P1** | H3 | POSVEL 速度断裂 | 设计级 | `pid_ref` 兼顾速度或加 LOGWARNING |
| **P2** | M1 | CAN 总线过载 | 中等 | 降低保活频率到 10Hz |
| **P2** | M2 | MIT STOP 语义不清 | 一行代码 | 改为发 DM_CMD_RESET_MODE |
| **P2** | M3 | 死循环饿死控制 | 一行代码 | 改为 return |
| **P2** | M4 | 多圈溢出 | 设计级 | 增加绝对位置校验 |
| **P3** | L1-L4 | 低危项 | 按需 | 见各节 |

### 总评

新版 dm_motor v2.0 的整体架构和代码质量**显著优于**旧版。发现的缺陷集中在防御性编程的缺失（限幅、越界检查、空指针处理）和与 DJI 驱动的 CAN 总线资源冲突上。核心的控制回路逻辑（串级 PID 流转、分组发送、反馈源切换）设计是正确的。P0/P1 缺陷修复后，驱动可以安全使用。CAN ID 冲突（C2）是唯一需要架构层面决策的问题，其他都是局部修补。
