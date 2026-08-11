#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "servo_motor.h"
#include "vofa.h"

static ServoInstance *servo_yaw_motor, *servo_pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static DMMotorInstance *motor_yaw, *motor_pitch;
static attitude_t *gimbal_IMU_data; // 云台IMU数据
#define ecd_maker 0//编码模式标志位
#define gyro_maker 1//陀螺仪模式标志位
// #define GIMBAL_ANGLE_GAIN 1.0f
// #define GIMBAL_RATE_GAIN 10.0f
//float yaw_angle_ref, pitch_angle_ref;
   static float relative_angle_gyro = 0;//陀螺仪相对角度反馈
 uint8_t mode_change_flag = 0;//模式转换标志位
  // count已移除,改为gyro_relative中以电机实例维度判断是否需要初始化



static void GimbalpositionControl(float *yaw_angle_cmd, float *pitch_angle_cmd);//编码器控制模式
static void GimbalIMUControl();//陀螺仪控制模式
static void GimbalModeControl();//模式转换控制
static void GimbalReset();//初始化云台
static void GimbalFreeMode();//自由模式
static void ecd_relative(DMMotorInstance *motor);//编码器转换成弧度制
static void gyro_relative(DMMotorInstance *motor, float gyro_angle_add, uint8_t goal);//角度制转换成弧度制

//static void GimbalpositionControl(float*yaw_angle_cmd,float*pitch_angle_cmd);
void GimbalInit()
{
    gimbal_IMU_data = INS_Init();
    Motor_Init_Config_s gimbal_Yaw_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50, // 4.5
                .Ki = 0, // 0
                .Kd = 5, // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
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
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
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
        DMMotorControlInit();
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 云台角度限幅
 *
 */
// static void GimbalAngleConstrain(float *yaw, float *pitch)
// {
//     if (*yaw >= YAW_MAX_ANGLE)
//         *yaw = YAW_MAX_ANGLE;
//     else if (*yaw <= YAW_MIN_ANGLE)
//         *yaw = YAW_MIN_ANGLE;

//     if (*pitch >= PITCH_MAX_ANGLE)
//         *pitch = PITCH_MAX_ANGLE;
//     else if (*pitch <= PITCH_MIN_ANGLE)
//         *pitch = PITCH_MIN_ANGLE;
// }
/**
 * @brief 云台初始化模式
 *
 */
static void GimbalReset()
{
    mode_change_flag=1;
   // float yaw_ecd_cmd = 0, pitch_ecd_cmd = 0;
    gyro_relative(motor_pitch, 0,1);
   // GimbalpositionControl(&yaw_ecd_cmd, &pitch_ecd_cmd);
    //yaw_ecd_cmd = (yaw_ecd_cmd - motor_yaw->measure.offset_ecd) * 2 / 8192 * 3.14;
    //pitch_ecd_cmd = (pitch_ecd_cmd - motor_pitch->measure.offset_ecd) * 2 / 8192 * 3.14;
    DMMotorSetRef(motor_yaw, 0);//初始化yaw
   // DMMotorSetRef(motor_pitch, 0, 0, 0, ecd_maker,1);
    DMMotorSetRef(motor_pitch, 0); motor_pitch->maker_flag = gyro_maker;
    //motor_pitch->raw_gyro = gimbal_IMU_data->Pitch;
    //motor_pitch->measure.offest_angle=motor_pitch ->raw_gyro;
  

}

/**
 * @brief 云台自由运动模式
 *
 */
static void GimbalFreeMode()
{
    static float yaw_ecd_accum = 0;         // Yaw 摇杆累积角度(度)
    static uint16_t yaw_base_ecd = 0;       // 进入 FREE_MODE 时的编码器值
    static uint8_t yaw_first_free = 1;      // 首次进入 FREE_MODE 标志

    gimbal_feedback_data.yaw_relative_angle = motor_yaw->measure.relative_angle;

    motor_pitch->raw_gyro = gimbal_IMU_data->Pitch;

    if (mode_change_flag == 1)
    {
        yaw_first_free = 1;
        motor_pitch->measure.offest_angle = motor_pitch->raw_gyro + 180.0f;
        mode_change_flag = 0;
    }

    /* ---- Yaw: 编码器闭环, 摇杆通过偏移 offset_ecd 控制角度 ---- */
    if (yaw_first_free) {
        yaw_base_ecd = motor_yaw->measure.ecd;   // 记录当前编码器位置
        yaw_ecd_accum = 0;
        yaw_first_free = 0;
    }
    yaw_ecd_accum += gimbal_cmd_recv.yaw_add_angle;         // 累积摇杆增量(度)
    {
        /* 从基准位置计算目标编码器值, 归一化到 [0, 8191] */
        int32_t delta = (int32_t)(yaw_ecd_accum * 8192.0f / 360.0f);
        int32_t target = (int32_t)yaw_base_ecd + delta;
        target = ((target % 8192) + 8192) % 8192;
        motor_yaw->measure.offset_ecd = (uint16_t)target;
    }
    ecd_relative(motor_yaw);
    DMMotorSetRef(motor_yaw, 0);       // Yaw: 编码器闭环 target=0

    /* ---- Pitch: IMU闭环(不变) ---- */
    float yaw_angle_cmd = 0, pitch_angle_cmd = 0;
    GimbalIMUControl(&yaw_angle_cmd, &pitch_angle_cmd);
    gyro_relative(motor_pitch, pitch_angle_cmd, 1);
    DMMotorSetRef(motor_pitch, 0); motor_pitch->maker_flag = gyro_maker;
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
static void GimbalpositionControl(float *yaw_angle_cmd, float *pitch_angle_cmd) // 位置控制
{
    if (yaw_angle_cmd == NULL || pitch_angle_cmd == NULL)
        return;

    // 计算云台角度指令
    *yaw_angle_cmd = gimbal_cmd_recv.yaw_ecd;
    *pitch_angle_cmd = gimbal_cmd_recv.pitch_ecd;
}

/**
 * @brief 根据模式选择控制方式
 *
 */
// static void GimbalModeControl()
// {
//     switch (gimbal_cmd_recv.gimbal_mode)
//     {
//     case GIMBAL_RESET:
//         GimbalReset(); /* code */
//         motor_yaw->stop_flag = MOTOR_ENALBED;
//         motor_pitch->stop_flag = MOTOR_ENALBED;
//         break;
//     case GIMBAL_FREE_MODE:
//         GimbalFreeMode(); /* code */
//         motor_yaw->stop_flag = MOTOR_ENALBED;
//         break;
//     case GIMBAL_NOMOVE:
//         motor_yaw->stop_flag = MOTOR_STOP;
//         motor_pitch->stop_flag = MOTOR_STOP;
//         break;
 
//     default:
//         break;
//     }

// }
static void GimbalModeControl()//模式转换控制
{
    if (gimbal_cmd_recv.gimbal_mode==GIMBAL_RESET)
    {
         GimbalReset(); /* code */
        motor_yaw->stop_flag = MOTOR_ENALBED;
        motor_pitch->stop_flag = MOTOR_ENALBED;
    }
    else if (gimbal_cmd_recv.gimbal_mode==GIMBAL_FREE_MODE)
    {
                GimbalFreeMode(); /* code */
        motor_yaw->stop_flag = MOTOR_ENALBED;
        motor_pitch->stop_flag = MOTOR_ENALBED;
    }
   else if (gimbal_cmd_recv.gimbal_mode==GIMBAL_NOMOVE)
   {
            motor_yaw->stop_flag = MOTOR_STOP;
        motor_pitch->stop_flag = MOTOR_STOP;
   }
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
        motor_pitch->raw_gyro = -4.5f;//pitch的初始化角度
    // float arr[2];
    // arr[0]=motor_yaw->measure.relative_angle_gyro*1000000;
    // arr[1]=0;
    DMMotorShootFlag(motor_pitch,gimbal_cmd_recv.shoot_flag);//不记得了应该没用
    DMMotorShootFlag(motor_yaw,0);        //发射前馈
    // vofa_justfloat_output(arr, 2 , &huart1);
    // Calculate_Angle(&yaw_angle_ref, motor_yaw);
    motor_yaw->measure.offset_ecd = 1045;//yaw的初始化编码值
    //motor_pitch->measure.offset_ecd = 5794;
    //motor_yaw->measure.gyro = gimbal_IMU_data->Gyro[2];//yaw的陀螺仪速度数据(禁用)
    motor_pitch->measure.gyro = gimbal_IMU_data->Gyro[0];//pitch的陀螺仪速度数据
    motor_pitch->measure.accel=gimbal_IMU_data->Accel[1];//pitch的加速度数据
    //motor_yaw->measure.accel = gimbal_IMU_data->Accel[2];//yaw的加速度数据(禁用)
    //motor_yaw->measure.gyro_angle = gimbal_IMU_data->Yaw;//yaw的角度数据(禁用)
    motor_pitch->measure.gyro_angle = gimbal_IMU_data->Pitch;//pitch的角度数据
    ecd_relative(motor_yaw);//初始化用
    ecd_relative(motor_pitch);//没什么用
   gimbal_feedback_data.yaw_relative_angle= motor_yaw->measure.relative_angle ;//传给地盘进行运动学解算
    GimbalModeControl();
    // ServoMotorControl(); // 驱动舵机转动
    //  推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}