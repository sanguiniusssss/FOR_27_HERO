#include "gimbal.h"
#include "robot_def.h"
#include "dm_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"


static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static DMMotorInstance *motor_yaw, *motor_pitch;
static attitude_t *gimbal_IMU_data; // 云台IMU数据
 uint8_t mode_change_flag = 0;//模式转换标志位


static void GimbalIMUControl(float *yaw_angle_relative, float *pitch_angle_relative);
static void gyro_relative(DMMotorInstance *motor, float gyro_angle_add, uint8_t goal);//角度制转换成弧度制

void GimbalInit()
{
    gimbal_IMU_data = INS_Init();
    Motor_Init_Config_s gimbal_Yaw_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50,
                .Ki = 0,
                .Kd = 1.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .MaxOut = 15000,
                .Derivative_LPF_RC = 0.005f,
            },
            .speed_PID = {
                .Kp = 15, // 4.5
                .Ki = 0, // 0
                .Kd = 0.01, // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 2000,
            },

            .gyro_PID = {
                .Kp = 15, // 0.4
                .Ki = 0, // 0
                .Kd = 0.01,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 2000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP,
        },
        .motor_type = M2006, // 达妙电机
    };
    Motor_Init_Config_s gimbal_Pitch_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 100, // 4.5
                .Ki = 0, // 0
                .Kd = 10, // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
            .speed_PID = {
                .Kp = 20,
                .Ki = 0,  // 0
                .Kd = 0.08,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 50,
            },
            .gyro_PID = {
                .Kp = 18, // 0.4
                .Ki = 0,   // 0
                .Kd = 0.1,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 50,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508, // 达妙电机
    };
     //@todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    gimbal_Yaw_config.can_init_config.tx_id = 1;
    gimbal_Yaw_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_yaw = DMMotorInit(&gimbal_Yaw_config, DJI_MODE);
    gimbal_Pitch_config.can_init_config.tx_id = 2;
    gimbal_Pitch_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_pitch = DMMotorInit(&gimbal_Pitch_config, DJI_MODE);
    motor_yaw->measure.offset_ecd = 1045;  // Yaw 物理零点偏移(一次标定)
        DMMotorControlInit();
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}
/**
 * @brief 云台初始化模式
 *
 */
static void GimbalReset()
{
    mode_change_flag = 1;

    /* Yaw: 复位到标定零点 */
    DMMotorSetRef(motor_yaw, 0);

    gyro_relative(motor_pitch, 0, 1);
    DMMotorSetRef(motor_pitch, 0);
}

/**
 * @brief 云台自由运动模式
 *
 */
static void GimbalFreeMode()
{
    static float  yaw_target_rad = 0;      // Yaw 目标角度(rad)
    static uint8_t yaw_first_free = 1;     // 首次进入 FREE_MODE 标志

    gimbal_feedback_data.yaw_relative_angle = motor_yaw->measure.relative_angle;

    motor_pitch->raw_gyro = gimbal_IMU_data->Pitch;

    if (mode_change_flag == 1)
    {
        yaw_first_free = 1;
        motor_pitch->measure.offest_angle = motor_pitch->raw_gyro + 180.0f;
        mode_change_flag = 0;
    }

    /* ---- Yaw: 直接计算目标角度(rad), PID 主动跟踪 ---- */
    if (yaw_first_free) {
        yaw_target_rad = motor_yaw->measure.relative_angle;  // 锁定当前位置
        yaw_first_free = 0;
    }
    yaw_target_rad += gimbal_cmd_recv.yaw_add_angle * (PI / 180.0f);
    /* 包裹到 ±π: ref 是角度不是累积圈数, dm_motor 负责映射到连续空间 */
    if (yaw_target_rad > PI)       yaw_target_rad -= 2.0f * PI;
    if (yaw_target_rad < -PI)      yaw_target_rad += 2.0f * PI;
    DMMotorSetRef(motor_yaw, yaw_target_rad);

    /* ---- Pitch: IMU闭环(不变) ---- */
    float yaw_angle_cmd = 0, pitch_angle_cmd = 0;
    GimbalIMUControl(&yaw_angle_cmd, &pitch_angle_cmd);
    gyro_relative(motor_pitch, pitch_angle_cmd, 1);
    DMMotorSetRef(motor_pitch, 0);
}
static void ecd_relative(DMMotorInstance *motor)//编码器转换成弧度制
{
     motor->measure.relative_ecd = motor->measure.ecd - motor->measure.offset_ecd;
    while (motor->measure.relative_ecd > HALF_ECD_RANGE)  // 4096
        motor->measure.relative_ecd -= ECD_RANGE;         // 8192
    while (motor->measure.relative_ecd < -HALF_ECD_RANGE)
        motor->measure.relative_ecd += ECD_RANGE;
    motor->measure.relative_angle=motor->measure.relative_ecd*MOTOR_ECD_TO_RAD;
}
static void gyro_relative(DMMotorInstance *motor, float gyro_angle_add, uint8_t goal)
{
    // 每个电机独立判断是否需要初始化:
    // 如果offest_angle与当前IMU角度(raw_gyro)差异>90度,说明尚未初始化或刚切换模式
    if (fabsf(motor->measure.offest_angle - motor->raw_gyro) > 90.0f)
        motor->measure.offest_angle = motor->raw_gyro;

  

    if(gyro_angle_add!=0)
    {
        motor->measure.offest_angle=motor->measure.offest_angle+gyro_angle_add; /* code */
    }
    if(motor->measure.offest_angle>180)
    {
        motor->measure.offest_angle=motor->measure.offest_angle-360;
    }
    else if (motor->measure.offest_angle<-180)
    {
        motor->measure.offest_angle=motor->measure.offest_angle+360;
    }
    if (goal==1)
    {
    if (motor->measure.offest_angle<-15)
    {
       motor->measure.offest_angle=-15; /* code */
    }
    else if (motor->measure.offest_angle>35)
    {
        motor->measure.offest_angle=35; /* code */
    }/* code */
    }
    


    
    motor->measure.relative_angle_gyro = motor->measure.gyro_angle - motor->measure.offest_angle;
    //motor->measure.relative_angle_gyro= relative_angle_gyro;
    if (motor->measure.relative_angle_gyro > HALF_GYRO_RANGE) // 180
    {
        motor->measure.relative_angle_gyro -= GYRO_RANGE; // 360
    }
    else if (motor->measure.relative_angle_gyro < -HALF_GYRO_RANGE)
    {
        motor->measure.relative_angle_gyro += GYRO_RANGE;
    }
    if (motor->measure.relative_angle_gyro > HALF_GYRO_RANGE) // 180
    {
        motor->measure.relative_angle_gyro -= GYRO_RANGE; // 360
    }
    else if (motor->measure.relative_angle_gyro < -HALF_GYRO_RANGE)
    {
        motor->measure.relative_angle_gyro += GYRO_RANGE;
    }
    motor->measure.relative_angle_gyro = motor->measure.relative_angle_gyro * 3.1415926/180;
}

static void GimbalIMUControl(float *yaw_angle_relative, float *pitch_angle_relative)//陀螺仪控制模式
{

    if (yaw_angle_relative == NULL || pitch_angle_relative == NULL)
        return;
    // 计算云台角速度指令

    *yaw_angle_relative = gimbal_cmd_recv.yaw_add_angle;
    *pitch_angle_relative = gimbal_cmd_recv.pitch_add_angle;

    // 设置电机参考值

    return;
}

static void GimbalModeControl()
{
    static gimbal_mode_e last_mode = GIMBAL_NOMOVE;

    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_RESET) {
        GimbalReset();
        motor_yaw->stop_flag = MOTOR_ENALBED;
        motor_pitch->stop_flag = MOTOR_ENALBED;
    }
    else if (gimbal_cmd_recv.gimbal_mode == GIMBAL_FREE_MODE) {
        /* 从其他模式切回 FREE 时重新锁定当前位置, 防止跳回旧目标 */
        if (last_mode != GIMBAL_FREE_MODE)
            mode_change_flag = 1;
        GimbalFreeMode();
        motor_yaw->stop_flag = MOTOR_ENALBED;
        motor_pitch->stop_flag = MOTOR_ENALBED;
    }
    else if (gimbal_cmd_recv.gimbal_mode == GIMBAL_NOMOVE) {
        motor_yaw->stop_flag = MOTOR_STOP;
        motor_pitch->stop_flag = MOTOR_STOP;
    }

    last_mode = gimbal_cmd_recv.gimbal_mode;
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
        motor_pitch->raw_gyro = -4.5f;//pitch的初始化角度
    DMMotorShootFlag(motor_pitch,gimbal_cmd_recv.shoot_flag);//不记得了应该没用
    DMMotorShootFlag(motor_yaw,0);        //发射前馈
    motor_pitch->measure.gyro = gimbal_IMU_data->Gyro[0];//pitch的陀螺仪速度数据
    motor_pitch->measure.accel=gimbal_IMU_data->Accel[1];//pitch的加速度数据
    motor_pitch->measure.gyro_angle = gimbal_IMU_data->Pitch;//pitch的角度数据
    ecd_relative(motor_yaw);//初始化用
    ecd_relative(motor_pitch);//没什么用
   gimbal_feedback_data.yaw_relative_angle= motor_yaw->measure.relative_angle ;//传给地盘进行运动学解算
    GimbalModeControl();
    //  推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}