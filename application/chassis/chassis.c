
#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "message_center.h"
#include "referee_task.h"
#include "elec_switch.h"
#include "ins_task.h"
#include "lowpass_filter.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include <stdint.h>
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_WHEEL_TRACK (WHEEL_TRACK / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */

static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static float wz_compensate; // 底盘陀螺仪PID补偿值

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static DMMotorInstance *motor_yaw;                                    // Yaw J4310 (CAN2)

static PIDInstance Chassis_wz_PID_Low, Chassis_wz_PID_High; // 底盘陀螺仪闭环控制 PID ,这里千万不能是指针，PIDInit()函数中没有 malloc 这一步

static Subscriber_t *gimbal_cmd_sub;                                  // 订阅云台控制命令(获取yaw增量)
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;                             // 云台控制命令

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float sin_theta=1;   // 设置底盘行进方向
static float cos_theta=0;   // 设置底盘行进方向
static float chassis_vx, chassis_vy; // 将云台系的速度投影到底盘
static float chassis_vx_m, chassis_vy_m, chassis_wz_m;// 经过坐标变换后的底盘速度   
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {// 电机控制设置初始化配置
            .angle_feedback_source = MOTOR_FEED,// 角度反馈来源
            .speed_feedback_source = MOTOR_FEED,// 速度反馈来源
            .outer_loop_type = SPEED_LOOP,// 外环类型
            .close_loop_type = SPEED_LOOP ,// 闭环类型
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;//
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    /* ---- Yaw J4310 达妙电机 (CAN2, tx_id=1, 参数复用 gimbal 原版) ---- */
    Motor_Init_Config_s yaw_cfg = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 1,
            .rx_id      = 0x301,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 50,
                .Ki            = 0,
                .Kd            = 5,
                .MaxOut        = 15000,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit
                               | PID_Derivative_On_Measurement,
            },
            .speed_PID = {
                .Kp            = 15,
                .Ki            = 0,
                .Kd            = 0.01f,
                .MaxOut        = 2000,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit
                               | PID_Derivative_On_Measurement,
            },
        },
        .controller_setting_init_config = {
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = J4310,
    };
    motor_yaw = DMMotorInit(&yaw_cfg, DM_DJI_MODE);
    DMMotorControlInit();

    chassis_sub    = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub    = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_sub = SubRegister("gimbal_cmd",  sizeof(Gimbal_Ctrl_Cmd_s));
}

#define LF_CENTER ((HALF_WHEEL_TRACK + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_WHEEL_TRACK - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_WHEEL_TRACK + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_WHEEL_TRACK - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/**
 * @brief 逆运动学解算，计算每个轮毂电机的输出
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumIKine()
{
     vt_rf = -chassis_vx + chassis_vy + wz_compensate * RF_CENTER; // 2
     vt_lf = chassis_vx + chassis_vy - wz_compensate * LF_CENTER;  // 1
     vt_lb = -chassis_vx + chassis_vy - wz_compensate * LB_CENTER; // 4
     vt_rb = chassis_vx + chassis_vy + wz_compensate * RB_CENTER;  // 3
}

/**
 * @brief 底盘控制量输出
 *
 */
static void ChassisOutput()
{
    // 完成加速度限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/* ======================== Yaw 角度控制 (J4310, CAN2) ======================== */

static float  yaw_ref = 0.0f;        // Yaw 绝对目标角度(度)
static uint8_t yaw_inited = 0;       // 首次初始化标志

static void YawControl(void)
{
    SubGetMessage(gimbal_cmd_sub, &gimbal_cmd_recv);

    /* 首次运行: 等待 CAN 反馈有效后, 用当前编码器位置初始化目标角度 */
    if (!yaw_inited) {
        if (motor_yaw->measure.total_angle == 0.0f &&
            motor_yaw->measure.ecd == 0 &&
            motor_yaw->measure.last_ecd == 0) {
            return; // CAN 反馈尚未到达, 等待下一帧
        }
        yaw_ref    = motor_yaw->measure.total_angle;
        yaw_inited = 1;
    }

    switch (gimbal_cmd_recv.gimbal_mode) {

    case GIMBAL_RESET:
        yaw_ref = motor_yaw->measure.total_angle;
        DMMotorEnable(motor_yaw);
        break;

    case GIMBAL_NOMOVE:
        DMMotorStop(motor_yaw);
        return;

    case GIMBAL_FREE_MODE:
    default:
        DMMotorEnable(motor_yaw);
        break;
    }

    /* 累加 DT7 摇杆增量 → 绝对角度目标 */
    yaw_ref += gimbal_cmd_recv.yaw_add_angle;

    /* 设定 Yaw 电机目标 */
    DMMotorSetRef(motor_yaw, yaw_ref);
}

/**
 * @brief 根据底盘模式选择控制方式
 *
 */
static void ChassisModeControl()
{
    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rb);

    // 根据底盘模式决定是否输出控制量
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE ||
        chassis_cmd_recv.chassis_mode == CHASSIS_NO_MOVE)
    {
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        wz_compensate = 0.0f;
        return;
    }

    // 底盘向右为左右正方向,向前为逆时针旋转为角度正方向;
    chassis_vx = -chassis_cmd_recv.vx;
    chassis_vy = -chassis_cmd_recv.vy;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

    // 根据控制模式设定底盘速度
    ChassisModeControl();

    // 根据控制模式进行逆运动学解算,计算底盘输出
    MecanumIKine();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    ChassisOutput();

    // Yaw 角度控制
    YawControl();

    // 反馈 Yaw 相对角度, 供 robot_cmd 做 yaw-follow 运动学
    chassis_feedback_data.yaw_relative_angle = motor_yaw->measure.relative_angle;

    // UI_INIT_SECOND();
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}