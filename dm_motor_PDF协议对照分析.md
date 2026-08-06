# dm_motor v2.0 — 对照官方手册的终极协议缺陷分析

> **资料清单**:
> - `DM-J4310-2EC V1.1` (2022.10.14) — MIT/POSVEL/VEL 手册
> - `EMIT版本说明.pdf` — EMIT（增强MIT）模式协议
> - `一拖四版本说明.pdf` — DJI 兼容模式（一拖四）协议
> - `DM 调试助手使用说明书 V1.4` (2024.03.06)
> - 代码: `dm_motor.c/h` v2.0 (2026-08-05)

---

## 一、一拖四（DJI 兼容）模式 — 官方协议 vs 代码

### 1.1 控制帧（MCU → 电机）

**官方手册规定:**

| 项目 | 组1 (电机1-4) | 组2 (电机5-8) |
|------|-------------|-------------|
| CAN 仲裁 ID | **0x3FE** | **0x4FE** |
| 控制频率 | 1000Hz | 1000Hz |
| 数据长度 | 8 bytes | 8 bytes |
| 字节布局 | 每2字节1颗电机(int16 电流值) | 同左 |

```
0x3FE帧 (电机1-4):
D[0-1] = motor1 current (int16)
D[2-3] = motor2 current (int16)
D[4-5] = motor3 current (int16)
D[6-7] = motor4 current (int16)

0x4FE帧 (电机5-8):
D[0-1] = motor5 current (int16)
D[2-3] = motor6 current (int16)
D[4-5] = motor7 current (int16)
D[6-7] = motor8 current (int16)
```

**代码现状:**

```c
// dm_motor.c:57-70
static CANInstance sender_assignment[6] = {
    [0] = {.txconf.StdId = 0x3FE, ...},  // CAN1
    [1] = {.txconf.StdId = 0x200, ...},  // CAN1 ← 🔴 手册没有!
    [2] = {.txconf.StdId = 0x2ff, ...},  // CAN1 ← 🔴 手册没有!
    [3] = {.txconf.StdId = 0x3FE, ...},  // CAN2
    [4] = {.txconf.StdId = 0x200, ...},  // CAN2 ← 🔴 手册没有!
    [5] = {.txconf.StdId = 0x2ff, ...},  // CAN2 ← 🔴 手册没有!
};
```

**🔴 错误:** 手册只定义了 **两个** 分组 ID——`0x3FE`（电机1-4）和 `0x4FE`（电机5-8）。代码却沿用了 DJI 电机驱动的 6 组设计（0x3FE / 0x200 / 0x2FF × 2 CAN），其中**0x200 和 0x2FF 不在达妙协议中**。

**正确应该是:**

```c
static CANInstance sender_assignment[4] = {  // 只需要4组,不是6组!
    [0] = {.txconf.StdId = 0x3FE, ...},  // CAN1 电机1-4
    [1] = {.txconf.StdId = 0x4FE, ...},  // CAN1 电机5-8
    [2] = {.txconf.StdId = 0x3FE, ...},  // CAN2 电机1-4
    [3] = {.txconf.StdId = 0x4FE, ...},  // CAN2 电机5-8
};
```

### 1.2 反馈帧（电机 → MCU）

**官方手册规定:**

| 项目 | 值 |
|------|-----|
| CAN 仲裁 ID | **`0x300 + motor_ID`** |
| 数据长度 | 8 bytes |

```
D[0]   D[1]   D[2]   D[3]   D[4]   D[5]   D[6]        D[7]
enc[H] enc[L] rpm[H] rpm[L] cur[H] cur[L] temperature  PCB_temp
```

| 字段 | 位宽 | 范围 | 含义 |
|------|------|------|------|
| encoder | 16 bits | 0-8191 | 编码器位置 |
| speed | 16 bits (int) | rpm | 电机转速 |
| current | 16 bits (int) | mA | 实际电流 |
| temperature | 8 bits | ℃ | 电机温度 |
| PCB_temp | 8 bits | ℃ | PCB/驱动板温度 |

**代码现状 — DMMotorDecode 解析:** ✅ 格式匹配

```c
measure->ecd      = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];      // ✅ enc[0-1]
measure->speed_aps = ... (int16_t)(rxbuff[2] << 8 | rxbuff[3]);   // ✅ rpm[2-3]
measure->real_current = ... (int16_t)(rxbuff[4] << 8 | rxbuff[5]);// ✅ cur[4-5]
measure->temperature = rxbuff[6];                                  // ✅ temp[6]
// rxbuff[7] — PCB温度，代码未使用
```

DMMotorDecode 的反馈解析是**正确的**。唯一小遗憾是未读取 rxbuff[7] 的 PCB 温度。

**代码现状 — 反馈 CAN ID（MotorSenderGrouping）:** 🔴 错误

```c
// dm_motor.c:209-219
case J4310:
case J3507:
    ...
    config->rx_id = 0x200 + motor_id + 1;  // ← 🔴 手册规定是 0x300 + ID!
```

**手册规定反馈 CAN ID = `0x300 + motor_ID`**。对于 tx_id=1 的电机（motor_ID=1），反馈应是 0x301。代码却计算为 `0x200 + 0 + 1 = 0x201`。

gimbal.c 中硬编码的 `rx_id = 0x301` 反而是正确的——但新版 `DMMotorInit` 中的 `MotorSenderGrouping()` **会覆盖**这个正确的值！

### 1.3 MotorSenderGrouping 完全对照

| 项目 | 手册规定 | 代码实现 | 结果 |
|------|---------|---------|------|
| 分组数/CAN | 2 组 (0x3FE, 0x4FE) | 3 组 (0x3FE, 0x200, 0x2FF) | 🔴 多了两个不存在的 ID |
| 反馈 rx_id | `0x300 + motor_ID` | `0x200 + motor_id + 1` | 🔴 偏移量错误 |
| 电机类型区分 | 未区分，统一处理 | J4310/J3507 vs J4340 不同分组 | ⚠️ 手册无此区分 |
| 电机1-4 分组 | 0x3FE | 0x3FE | ✅ |
| 电机5-8 分组 | **0x4FE** | 0x3FE（CAN2上） | 🔴 应是0x4FE |

---

## 二、EMIT（增强MIT）模式 — 代码完全未实现

### 2.1 官方协议

**EMIT 是达妙特有的新模式**，使用简化的 CAN 帧格式：

| 项目 | 值 |
|------|-----|
| CAN 仲裁 ID | **`0x300 + motor_ID`** |
| 数据长度 | 8 bytes |

```
D[0-1]  D[2-3]  D[4-5]  D[6-7]
p_des   v_des   i_des   (保留?)
```

| 字段 | 位宽 | 范围 | 含义 |
|------|------|------|------|
| p_des | 16 bits | rad | 目标位置 |
| v_des | 16 bits | 0-10000 → 0-100 rad/s | 目标速度 |
| i_des | 16 bits | 0-10000 → 0-1.0 | 目标电流（归一化） |

### 2.2 与 MIT 模式的关键区别

| 项目 | 标准 MIT | EMIT |
|------|---------|------|
| CAN ID | `motor_ID` | `0x300 + motor_ID` |
| 打包方式 | 复杂位域 (16+12+12+12+12) | 简单 16+16+16 |
| Kp/Kd | 可配置 | 无（驱动侧自适应） |
| 数据域 | p_des+v_des+t_ff+Kp+Kd | p_des+v_des+i_des |

### 2.3 代码状态

dm_motor v2.0 中**完全没有 EMIT 模式**。`DMControl_Mode_e` 中没有 `DM_EMIT_MODE`。

---

## 三、⚠️ CAN ID 冲突全景图（对照手册修正后）

### 各模式的 CAN ID 分配（手册规定）

```
模式        控制帧(MCU→电机)        反馈帧(电机→MCU)
──────────────────────────────────────────────────
MIT         motor_ID                MST_ID (default 0)
POSVEL      0x100 + motor_ID        MST_ID (default 0)
VEL         0x200 + motor_ID        MST_ID (default 0)
EMIT        0x300 + motor_ID        ?
一拖四       0x3FE / 0x4FE (分组)    0x300 + motor_ID
```

### 🔴 冲突1: VEL 控制帧 vs EMIT / 一拖四反馈

| 冲突 | CAN ID | 方向 |
|------|--------|------|
| VEL 控制 | `0x200 + motor_ID` | MCU→电机 |
| 一拖四反馈 | `0x300 + motor_ID` | 电机→MCU |

不直接冲突（方向不同），但如果某电机同时需要 VEL 控制和一拖四反馈，需要两个 CAN 过滤器。

### 🔴 冲突2: EMIT 控制帧 vs 一拖四反馈帧

| 冲突 | CAN ID | 方向 |
|------|--------|------|
| EMIT 控制 | `0x300 + motor_ID` | MCU→电机 |
| 一拖四反馈 | `0x300 + motor_ID` | 电机→MCU |

**同一个 CAN ID 两个方向各用**。这在 CAN 协议层是合法的（TX/RX 分开），但代码中如果同时有 EMIT 模式和 DJI_MODE 的电机，需要确保 CAN 过滤器不会混淆。

### 🔴 冲突3: dm_motor 的 0x200/0x2FF vs DJI 驱动

| CAN ID | dm_motor 声称的用途 | DJI 驱动实际用途 | 
|--------|--------------------|--------------------|
| 0x200 | DM 分组 (错误) | DJI M2006/M3508 分组 |
| 0x2FF | DM 分组 (错误) | DJI GM6020 分组 |

**按手册修正 dm_motor 后（只用 0x3FE + 0x4FE），这个冲突自然消失**。

---

## 四、命令帧 — 手册未描述

EMIT 和一拖四手册**都没有描述** 0xFC/0xFD/0xFE/0xFB 命令帧。这些命令可能来自：
- 达妙调试助手的通信协议（在 `DM 调试助手使用说明书 V1.4` 中）
- 固件默认支持的扩展命令

---

## 五、修正方案

### 5.1 sender_assignment 修正

```c
// 修正前 (6组, 含不存在的ID):
static CANInstance sender_assignment[6] = { ... 0x3FE, 0x200, 0x2FF ... };

// 修正后 (4组, 完全符合手册):
static CANInstance sender_assignment[4] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x3FE, ...},  // CAN1 电机1-4
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x4FE, ...},  // CAN1 电机5-8
    [2] = {.can_handle = &hcan2, .txconf.StdId = 0x3FE, ...},  // CAN2 电机1-4
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x4FE, ...},  // CAN2 电机5-8
};
static uint8_t sender_enable_flag[4] = {0};
```

`sender_enable_flag` 也需要从 `[6]` 缩小为 `[4]`。

`DMMotorControl()` 末尾的统一发送循环也要改：
```c
for (size_t i = 0; i < 4; i++) { ... }  // 原来是 i < 6
```

### 5.2 MotorSenderGrouping 修正

```c
// 修正前:
config->rx_id = 0x200 + motor_id + 1;  // ← 错误!

// 修正后:
config->rx_id = 0x300 + motor_id + 1;  // 0x300 + motor_ID
//               ↑↑↑
```

或者直接使用 `config->tx_id`:
```c
config->rx_id = 0x300 + config->tx_id;  // tx_id 就是 motor_ID
```

### 5.3 分组逻辑修正

手册不区分 J4310/J3507/J4340 的分组规则（一拖四模式下统一处理），但 DM 不同类型的电机可能需要不同的电流缩放。当前的类型区分可以保留，但分组 ID 不应因类型而异——**一拖四只有 0x3FE 和 0x4FE**：

```c
static void MotorSenderGrouping(DMMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1;

    // 一拖四只有两种分组:
    if (motor_id < 4) {
        motor->message_num = motor_id;                              // 组内 0-3
        motor->sender_group = (config->can_handle == &hcan1) ? 0 : 2; // CAN1组0 / CAN2组2
    } else if (motor_id < 8) {
        motor->message_num = motor_id - 4;                          // 组内 0-3
        motor->sender_group = (config->can_handle == &hcan1) ? 1 : 3; // CAN1组1 / CAN2组3
    } else {
        LOGERROR("[dm_motor] Motor ID %d out of range (max 8)!", config->tx_id);
        return;
    }

    // 反馈 ID = 0x300 + motor_ID
    config->rx_id = 0x300 + config->tx_id;

    sender_enable_flag[motor->sender_group] = 1;
    
    // ID 冲突检测 (保持不变)
    ...
}
```

### 5.4 后果：与 DJI 驱动不再冲突

修正后，dm_motor 使用的 CAN ID 为：
- 控制: `0x3FE`, `0x4FE`
- 反馈: `0x300 + ID`

DJI 驱动 `dji_motor.c` 使用的 CAN ID 为：
- 控制: `0x1FF`, `0x200`, `0x2FF`
- 反馈: `0x200 + ID`, `0x204 + ID`

**两组 CAN ID 完全无重叠**。之前分析的"🔴 CRITICAL-2: CAN ID 冲突"问题，在本修正后自动解决。

---

## 六、缺陷等级更新

根据官方手册修正后的严重度评定：

| ID | 缺陷 | 原评级 | 手册对照后 | 说明 |
|----|------|--------|----------|------|
| sender_assignment CAN ID | 0x200/0x2FF 不在 DM 协议中 | — | **🔴 CRITICAL 新增** | 发到不存在的 ID，电机收不到 |
| MotorSenderGrouping rx_id | 0x200+id 而非 0x300+id | — | **🔴 CRITICAL 新增** | 反馈帧注册到错误的 CAN ID |
| C1 | 输出无 int16 限幅 | 🔴 P0 | 🔴 P0 维持 | |
| H1 | 数组越界 | 🟠 P1 | 🟠 P1 维持 | |
| H2 | 空指针回退飞车 | 🟠 P1 | 🟠 P1 维持 | |
| H3 | POSVEL 速度断裂 | 🟠 P1 | 🟠 P1 维持 | |
| M1 | CAN 过载 | 🟡 P2 | 🟡 P2 维持 | 减少到4组后有所缓解 |
| EMIT 模式缺失 | `DM_EMIT_MODE` 未实现 | — | **🟡 P2 新增** | 功能缺失，非 bug |

---

## 七、总结

**根据官方手册，dm_motor v2.0 的核心问题只有一个根源**：代码在 DJI_MODE 下错误地照搬了 DJI 电机驱动的 CAN ID 分配方案（0x3FE / 0x200 / 0x2FF × 2CAN），而没有按照达妙一拖四手册规定的 0x3FE / 0x4FE 方案。反馈 rx_id (0x200+id) 也同样照搬了 DJI 驱动的公式，正确的应该是 0x300+id。

好消息是：**DMMotorDecode 的反馈解析是正确的**，MIT/POSVEL/VEL 的 tx_id 偏移和数据打包也全部正确。只需修正 sender_assignment 的 CAN ID 和 MotorSenderGrouping 的 rx_id 计算，DJI_MODE 就能正常工作。
