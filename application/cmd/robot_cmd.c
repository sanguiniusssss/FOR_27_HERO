// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "upper.h"
// module
#include "remote_control.h"
#include "self_controller.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信

#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD


#define CAN_LSY_ID 0x312
uint8_t gimbal_can_send_data[8];
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Publisher_t *upper_cmd_pub;           // 上层机构控制消息发布者
static Subscriber_t *upper_feed_sub;         // 上层机构反馈信息订阅者
static Upper_Ctrl_Cmd_s upper_cmd_send;      // 传递给上层机构的控制信息
static Upper_Upload_Data_s upper_fetch_data; // 从上层机构获取的反馈信息

static TickType_t dial_time_start; // 计时开始时的时间
static TickType_t dial_time_now;   // 当前时刻时间
static uint8_t dial_press_flag;    // 拨轮按下标志位
static upper_mode_e upper_last_mode;

static Robot_Status_e robot_state; // 机器人整体工作状态

static Self_Cntlr_s *self_ctrl_data; // 自定义控制器数据接收

static PIDInstance *pid_chassis_wz;
static PID_Init_Config_s *config_chassis_wz;

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
float chassis_wz_ref = 0;
uint8_t shoot_start_flag=0;
uint8_t leader_flag=0;
uint8_t shoot_flag=0;
uint8_t shoot_num=0;
uint8_t shoot_change=0;
uint16_t shoot_stop_num=0;
uint8_t shoot_change_flag=0;
float shoot_p=1.00;
uint8_t  Flag_z=0;
uint8_t Flag_x=0;
uint16_t shoot_last_mode=0;
void RobotCMDInit()
{
       config_chassis_wz = malloc(sizeof(PID_Init_Config_s));
    config_chassis_wz->Kp = 15000;
    config_chassis_wz->Ki = 0;
    config_chassis_wz->Kd = 0;
    config_chassis_wz->Improve = PID_Integral_Limit | PID_Derivative_On_Measurement;
    config_chassis_wz->MaxOut = 15000;
    config_chassis_wz->IntegralLimit = 10000;
    config_chassis_wz->Kf = 0;
    config_chassis_wz->DeadBand=0;
    config_chassis_wz->Ref_FF=0;
 
  pid_chassis_wz = malloc(sizeof(PIDInstance));

   

    PIDInit(pid_chassis_wz, config_chassis_wz);
#if defined(USE_DT7) || defined(USE_FS)
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#endif
#ifdef USE_VT13
    rc_data = RemoteControlInit(&huart6); // VT13使用图传串口进行数据传输
#endif

    //self_ctrl_data = SelfCntlrInit(&huart6);

     //vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    // upper_cmd_pub = PubRegister("upper_cmd", sizeof(Upper_Ctrl_Cmd_s));
    // upper_feed_sub = SubRegister("upper_feed", sizeof(Upper_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };


    cmd_can_comm = CANCommInit(&comm_conf);



#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;
    // gimbal_cmd_send.yaw_fixed_angle = 0;
    gimbal_cmd_send.yaw_free_angle = 220;
    // gimbal_cmd_send.pitch_free_angle = 90;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 将当前位姿更新到发送端，使后续操作在当前位姿上执行
 * @note 注意这个函数不能被一直调用，否则会与键鼠/遥控器的控制冲突
 *
 */
static void CmdRecvUpdate()
{
    upper_cmd_send.joint_data.yaw1 = upper_fetch_data.joint_data.yaw1;
    upper_cmd_send.joint_data.yaw2 = upper_fetch_data.joint_data.yaw2;
    upper_cmd_send.joint_data.yaw3 = upper_fetch_data.joint_data.yaw3;
    upper_cmd_send.joint_data.roll_differ = upper_fetch_data.joint_data.roll_differ;
    upper_cmd_send.joint_data.pitch_differ = upper_fetch_data.joint_data.pitch_differ;
    upper_cmd_send.joint_data.lift_dist = upper_fetch_data.joint_data.lift_dist;
}

/**
 * @brief 云台控制数据更新
 *
 */
static void GimbalSendUpdate()
{
    gimbal_cmd_send.yaw1_angle = upper_cmd_send.joint_data.yaw1;
}

/**
 * @brief 底盘控制数据更新,用于刷新UI
 *
 */
// static void ChassisSendUpdate()
// {
//     chassis_cmd_send.gimbal_mode = gimbal_cmd_send.gimbal_mode;
// }

#ifdef USE_DT7
/**
 * @brief 控制输入为DT7遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // static uint8_t test_flag = 1;
    // if (upper_cmd_send.upper_mode != UPPER_CALI)
    //{
    

    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[下]
    {
        
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        
        if (rc_data[TEMP].rc.dial < -450)
        {       //CAN_cmd_lsy(0, 0, 0, 0);
             chassis_cmd_send.wz = -2000.0f; // 逆时针旋转
        }
        else if (rc_data[TEMP].rc.dial > 600)
        { //CAN_cmd_lsy(0, 1, 0, 0);
             chassis_cmd_send.wz = 2000.0f; // 顺时针旋转
        }
        else if (rc_data[TEMP].rc.dial == 0)
        {
            chassis_wz_ref = gimbal_fetch_data.yaw_relative_angle;
            chassis_cmd_send.wz = PIDCalculate(pid_chassis_wz, chassis_wz_ref, 0);
        }
        chassis_cmd_send.vx = -20.0f * (float)rc_data[TEMP].rc.rocker_r_;              // _水平方向
        chassis_cmd_send.vy = 20.0f * (float)rc_data[TEMP].rc.rocker_r1;               // |竖直方向
        gimbal_cmd_send.yaw_add_angle = -(float)rc_data[TEMP].rc.rocker_l_ * 0.001f;   // 水平方向
        gimbal_cmd_send.pitch_add_angle = (float)rc_data[TEMP].rc.rocker_l1 * 0.0005f; // 竖直方向
        chassis_cmd_send.relative_angle = gimbal_fetch_data.yaw_relative_angle;
             if (switch_is_up(rc_data[TEMP].rc.switch_right))
     {

        if(shoot_start_flag==1)
        {   
            shoot_cmd_send.shoot_single_flag=1;
           shoot_start_flag=0;
        }

            shoot_cmd_send.shoot_mode = SHOOT_ON; 
            shoot_cmd_send.friction_mode =FRICTION_ON;
           shoot_cmd_send.bullet_speed=SMALL_AMU_15;
            chassis_cmd_send.pump_mode = shoot_fetch_data.shoot_state;
     }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
        {
        // shoot_cmd_send.shoot_mode = SHOOT_ON;
        // shoot_cmd_send.friction_mode =FRICTION_ON;
        // shoot_cmd_send.bullet_speed=SMALL_AMU_15;
         //shoot_cmd_send.bullet_speed=SMALL_AMU_18;
        shoot_cmd_send.shoot_single_flag=0; 
        shoot_start_flag=1;
        chassis_cmd_send.pump_mode = 0;

        }
    }

    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[中]
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_RESET;
        //gimbal_cmd_send.yaw_ecd = GIMBAL_YAW_ECD;
        //gimbal_cmd_send.pitch_ecd = GIMBAL_PITCH_ECD;
    }
    // else if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[下] 部署模式
    // {
    //     chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    //     gimbal_cmd_send.yaw_add_angle = -(float)rc_data[TEMP].rc.rocker_l_ * 0.001f;   // 水平方向
    //     gimbal_cmd_send.pitch_add_angle = -(float)rc_data[TEMP].rc.rocker_l1 * 0.000001f; // 竖直方向
    // }

   else if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_NOMOVE;
        //shoot_cmd_send.shoot_mode = SHOOT_OFF;
        //chassis_cmd_send.pump_mode = MOTOR_STOP;
    }
    // if (switch_is_down(rc_data[TEMP].rc.switch_right))
    // {

    //    /* code */
    // }
    // // else
    // {
       
    // }
    


    // else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    //{

    // }射击用

    //}
    // else
    // {
    //     if (upper_fetch_data.action_step == 0)
    //     {
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         CmdRecvUpdate();
    //     }
    // }
}
#endif // USE_DT7

#ifdef USE_VT13
/**
 * @brief 控制输入为VT13遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
   // static uint8_t test_flag = 1;
    // if (upper_cmd_send.upper_mode != UPPER_CALI)
    // {
        // if (rc_data[TEMP].rc.Stop_button_count % 2 == 0) // 模式切换按键按下偶数次
        // {
            // 控制底盘运行模式

           if (switch_is_S(rc_data[TEMP].rc.switch_mid)) // 中置开关状态[S],底盘正常行进
            {
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
             chassis_cmd_send.vx = -70.0f * (float)rc_data[TEMP].rc.rocker_r_;  // _水平方向
            chassis_cmd_send.vy = 70.0f * (float)rc_data[TEMP].rc.rocker_r1;  // |竖直方向

            gimbal_cmd_send.yaw_add_angle = -0.001f * (float)rc_data[TEMP].rc.rocker_l_;
             gimbal_cmd_send.pitch_add_angle = 0.0004f * (float)rc_data[TEMP].rc.rocker_l1;
            chassis_cmd_send.relative_angle = gimbal_fetch_data.yaw_relative_angle;
             if (rc_data[TEMP].rc.dial==0)
             {
                                        chassis_wz_ref = gimbal_fetch_data.yaw_relative_angle;
            chassis_cmd_send.wz = PIDCalculate(pid_chassis_wz, chassis_wz_ref, 0);/* code */
             }
             else
             {

                chassis_cmd_send.wz=rc_data[TEMP].rc.dial*10;
             }
             
                         if (rc_data[TEMP].rc.Stop_button_count%2 == 1)
            {
                        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
                        gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
            gimbal_cmd_send.yaw_add_angle = -0.001f * (float)rc_data[TEMP].rc.rocker_l_;
             gimbal_cmd_send.pitch_add_angle = 0.0004f * (float)rc_data[TEMP].rc.rocker_l1; /* code */
            }

            
            }
            else if (switch_is_N(rc_data[TEMP].rc.switch_mid))
            {
                         chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
                         gimbal_cmd_send.gimbal_mode = GIMBAL_RESET;
                                    chassis_cmd_send.vx = 0.0f;
                                    chassis_cmd_send.vy = 0.0f;
                                    chassis_cmd_send.wz = 0.0f;
            }
            // if(rc_data[TEMP].rc.Stop_button == 1)
            // {
            //     chassis_cmd_send.Super_flag = 1;
            // }
            // else if (rc_data[TEMP].rc.Stop_button == 0)
            // {
            //      chassis_cmd_send.Super_flag = 0;
            // }

             if(rc_data[TEMP].rc.trigger_button == 1)
            {
                chassis_cmd_send.Super_flag = 0;
                        shoot_cmd_send.shoot_single_flag=0; 
                        shoot_start_flag=1;
                        chassis_cmd_send.pump_mode = 0;  /* code *///拨弹电机的标志位
            }
            else if (rc_data[TEMP].rc.trigger_button == 0)
            {
                if(shoot_start_flag==1)
                {   
                    shoot_cmd_send.shoot_single_flag=1;
                    shoot_start_flag=0;
                }
                    chassis_cmd_send.pump_mode = shoot_fetch_data.shoot_state;/* code */ /* code */
                if(chassis_cmd_send.pump_mode==1)
                {
                    shoot_stop_num++;
                    if(shoot_stop_num>=200)
                    {
                        shoot_stop_num=0;
                        shoot_cmd_send.shoot_single_flag=0; 
                    }
                }
            }
       shoot_cmd_send.shoot_num=shoot_p;

            if(rc_data[TEMP].rc.Custom_button_left_count%2 == 1)
            {
                shoot_cmd_send.friction_mode =FRICTION_ON;
                    shoot_cmd_send.shoot_mode = SHOOT_ON;
                     shoot_cmd_send.bullet_speed=SMALL_AMU_15; 
            }
            else if(rc_data[TEMP].rc.Custom_button_left_count%2 == 0)
            {
                    shoot_cmd_send.shoot_mode = SHOOT_ON;
                         shoot_cmd_send.friction_mode =FRICTION_OFF;  /* code */
                     chassis_cmd_send.pump_mode = 0;
            }

        // else if (rc_data[TEMP].rc.Stop_button_count % 2 == 1) // 模式切换按键按下奇数次
        // {
        //     upper_cmd_send.upper_mode = UPPER_SINGLE_MOTOR;
        //     gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;

            // if (switch_is_S(rc_data[TEMP].rc.switch_mid)) // 中置开关状态[S] ，抬升+yaw1+yaw2
            // {                                             // yaw1,yaw2有传动齿轮，方向相反
            //     upper_cmd_send.joint_data.yaw1 -= 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
            //     upper_cmd_send.joint_data.lift_dist += 0.002f * (float)rc_data[TEMP].rc.rocker_l1;
            //     upper_cmd_send.joint_data.yaw2 -= 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
            // }
            // else if (switch_is_N(rc_data[TEMP].rc.switch_mid)) // 中置开关状态[N] ，yaw3+差速器
            // {                                                  // yaw3有传动齿轮，方向相反
            //     upper_cmd_send.joint_data.yaw3 -= 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
            //     upper_cmd_send.joint_data.roll_differ += 0.001f * (float)rc_data[TEMP].rc.rocker_r_; // 待改动
            //     upper_cmd_send.joint_data.pitch_differ += 0.001f * (float)rc_data[TEMP].rc.rocker_r1;
            // }
        //}
        // if (rc_data[TEMP].rc.trigger_button_count % 2 == 1) // 扳机键按下奇数次进入自定义控制器模式
        // {
        //     upper_cmd_send.upper_mode = UPPER_EXCHANGE;

        //     upper_cmd_send.ctrl_data.lift_dist = self_ctrl_data->lift_dist;
        //     upper_cmd_send.ctrl_data.yaw1 = self_ctrl_data->yaw1;
        //     upper_cmd_send.ctrl_data.yaw2 = self_ctrl_data->yaw2;
        //     upper_cmd_send.ctrl_data.yaw3 = self_ctrl_data->yaw3;
        //     upper_cmd_send.ctrl_data.pitch = self_ctrl_data->pitch;
        //     upper_cmd_send.ctrl_data.roll = self_ctrl_data->roll;
        // }
        // 真空泵控制,拨轮向上打为负,向下为正
        // if (rc_data[TEMP].rc.dial < -100) // 向上打开/关闭真空泵
        // {
        //     chassis_cmd_send.pump_mode = VALVE_ALL_OPEN;
        // }
        // else if (rc_data[TEMP].rc.dial > 100)
        // {
        //     chassis_cmd_send.pump_mode = VALVE_ALL_CLOSE;
        // }
        // // 自定义按键左按下进入校准模式
        // if (rc_data[TEMP].rc.Custom_button_left)
        // {
        //     upper_cmd_send.upper_mode = UPPER_CALI;
        // }
    }
    // else
    // {
    //     if (upper_fetch_data.action_step == 0)
    //     {
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         CmdRecvUpdate();
    //     }
    // }
//}
#endif // USE_VT13

#ifdef USE_FS
/**
 * @brief 控制输入为富斯遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{

    if (switch_is_up(rc_data[TEMP].rc.switch_a)&&switch_is_up(rc_data[TEMP].rc.switch_b)&&switch_is_up(rc_data[TEMP].rc.switch_c)&&switch_is_up(rc_data[TEMP].rc.switch_d))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_RESET;
        /* code */
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_a)&&switch_is_up(rc_data[TEMP].rc.switch_d))
    {
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
        chassis_cmd_send.vx = -80.0f * (float)rc_data[TEMP].rc.rocker_r_;              // _水平方向
        chassis_cmd_send.vy = 80.0f * (float)rc_data[TEMP].rc.rocker_r1;               // |竖直方向
        gimbal_cmd_send.yaw_add_angle = -(float)rc_data[TEMP].rc.rocker_l_ * 0.001f;   // 水平方向
        gimbal_cmd_send.pitch_add_angle = (float)rc_data[TEMP].rc.rocker_l1 * 0.0005f; // 竖直方向
                    chassis_wz_ref = gimbal_fetch_data.yaw_relative_angle;
            chassis_cmd_send.wz = PIDCalculate(pid_chassis_wz, chassis_wz_ref, 0);
        if (switch_is_up(rc_data[TEMP].rc.switch_b))
        {
            shoot_cmd_send.shoot_mode = SHOOT_ON; 
            shoot_cmd_send.friction_mode =FRICTION_ON;
           shoot_cmd_send.bullet_speed=SMALL_AMU_18; /* code */
        }
        else if (switch_is_down(rc_data[TEMP].rc.switch_b))
        {
            shoot_cmd_send.shoot_mode = SHOOT_ON; 
            shoot_cmd_send.friction_mode =FRICTION_ON;
           shoot_cmd_send.bullet_speed=SMALL_AMU_15; /* code */
        }
        if (switch_is_up(rc_data[TEMP].rc.switch_c))
        {
        shoot_cmd_send.shoot_single_flag=0; 
        shoot_start_flag=1;
        chassis_cmd_send.pump_mode = 0;/* code */
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_c))
        {
                    if(shoot_start_flag==1)
        {   
            shoot_cmd_send.shoot_single_flag=1;
           shoot_start_flag=0;
        }
                  chassis_cmd_send.pump_mode = shoot_fetch_data.shoot_state;
        }    
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_d))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_NOMOVE;
    }
    
        // if (switch_is_mid(rc_data[TEMP].rc.switch_c)) // C 拨杆中置，遥控模式
        // {
        //     if (switch_is_up(rc_data[TEMP].rc.switch_a)) // A 拨杆上拨，底盘模式
        //     {
        //         // 控制底盘运行模式
        //         if (switch_is_up(rc_data[TEMP].rc.switch_b)) // B 拨杆上拨,底盘正常行进
        //         {

        //         }

        //     }
        //     else if (switch_is_down(rc_data[TEMP].rc.switch_a)) // A 拨杆下拨，机械臂模式
        //     {


        //         if (switch_is_up(rc_data[TEMP].rc.switch_b)) // B 拨杆上拨 ，抬升+yaw1+yaw2
        //         {                                            // yaw1,yaw2有传动齿轮，方向相反

        //         }
        //         else if (switch_is_down(rc_data[TEMP].rc.switch_b)) // B 拨杆下拨 ，yaw3+差速器
        //         {                                                   // yaw3有传动齿轮，方向相反

        //         }
        //     }
        // }
        // if (switch_is_up(rc_data[TEMP].rc.switch_c)) // C 拨杆上拨,自定义控制器模式
        // {

        // }
        // if ((fabsf(rc_data[TEMP].rc.knob_A - rc_data[LAST].rc.knob_A) > 50) && (rc_data[LAST].rc.knob_A != 0)) // 扭动旋钮 A 进入校准模式
        // {

        // }
        // // 真空泵控制
        // if (switch_is_down(rc_data[TEMP].rc.switch_d)) // D 拨杆上拨打开真空泵
        // {
   
        // }
        // else if (switch_is_up(rc_data[TEMP].rc.switch_d)) // D 拨杆上拨关闭真空泵
        // {

        // }
        // 进入校准模式
    }

#endif // USE_FS

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
#ifndef USE_FS // 富斯遥控器没有键鼠功能
    /***********************************************   此处为基本动作控制   ***********************************************/



    // if (!(rc_data[TEMP].key[KEY_PRESS].shift) && !(rc_data[TEMP].key[KEY_PRESS].ctrl))
    // //  W/S  //  A/D  //  Q/E  //
    // //  前后 //  左右  // 旋转  //
    // {
        if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE&& gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE)
        {
            chassis_cmd_send.vx = -130000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].a - (float)rc_data[TEMP].key[KEY_PRESS].d); // _水平方向
            chassis_cmd_send.vy = 130000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].w - (float)rc_data[TEMP].key[KEY_PRESS].s); // |竖直方向
                gimbal_cmd_send.yaw_add_angle = -0.003f * rc_data[TEMP].mouse.x;
                gimbal_cmd_send.pitch_add_angle = 0.003f * rc_data[TEMP].mouse.y;
                            chassis_wz_ref = gimbal_fetch_data.yaw_relative_angle;
                                    chassis_cmd_send.relative_angle = gimbal_fetch_data.yaw_relative_angle;


            if ( rc_data[TEMP].key[KEY_PRESS].shift)
            {
               chassis_cmd_send.wz = -20000.0f;
            }     
            else if(!(rc_data[TEMP].key[KEY_PRESS].shift))
            {
            chassis_cmd_send.wz = PIDCalculate(pid_chassis_wz, chassis_wz_ref, 0);
            }
            if (rc_data[TEMP].key[KEY_PRESS].e)
            {
               HAL_NVIC_SystemReset (); /* code */
            }
            
                if (rc_data[TEMP].mouse.press_l)
                {
                        shoot_cmd_send.shoot_single_flag=0; 
                        shoot_start_flag=1;
                        //chassis_cmd_send.pump_mode = 1;  /* code *///拨弹电机的标志位
                }
                else if (!(rc_data[TEMP].mouse.press_l))
                {
                     if(shoot_start_flag==1)
                {   
                    shoot_cmd_send.shoot_single_flag=1;
                    shoot_start_flag=0;
                }
                    chassis_cmd_send.pump_mode = shoot_fetch_data.shoot_state;/* code */
                }
                if(chassis_cmd_send.pump_mode==1)
                {
                    shoot_stop_num++;
                    if(shoot_stop_num>=500)
                    {
                        shoot_stop_num=0;
                        shoot_cmd_send.shoot_single_flag=0; 
                    }
                }
                if (rc_data[TEMP].mouse.press_m&&shoot_flag==1)
                {
                    shoot_flag=0;
                     shoot_num++;      /* code */
                }
                else if (!(rc_data[TEMP].mouse.press_m))
                {
                    shoot_flag=1;
                }
                if ( shoot_num%2==0)
                {
                    shoot_cmd_send.shoot_mode = SHOOT_ON;
                         shoot_cmd_send.friction_mode =FRICTION_OFF;  /* code */
                     chassis_cmd_send.pump_mode = 0;
                }
                else if ( shoot_num%2==1)
                {
                    shoot_cmd_send.friction_mode =FRICTION_ON;
                    shoot_cmd_send.shoot_mode = SHOOT_ON;
                    // shoot_cmd_send.bullet_speed=SMALL_AMU_30;  /* code */
                     
                }
                // if (rc_data[TEMP].key[KEY_PRESS].q)
                // {
                //     chassis_cmd_send.Super_flag = 1; /* code */
                // }
                // else if (!(rc_data[TEMP].key[KEY_PRESS].q))
                // {
                //      chassis_cmd_send.Super_flag = 0; /* code */
                // }
}
if (chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE&& gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE)
        {
            gimbal_cmd_send.yaw_add_angle = -0.006f * rc_data[TEMP].mouse.x;
                gimbal_cmd_send.pitch_add_angle = 0.006f * rc_data[TEMP].mouse.y;
          if (rc_data[TEMP].key[KEY_PRESS].e)
            {
               HAL_NVIC_SystemReset (); /* code */
            }
            
                if (rc_data[TEMP].mouse.press_l)
                {
                        shoot_cmd_send.shoot_single_flag=0; 
                        shoot_start_flag=1;
                        //chassis_cmd_send.pump_mode = 1;  /* code *///拨弹电机的标志位
                }
                else if (!(rc_data[TEMP].mouse.press_l))
                {
                     if(shoot_start_flag==1)
                {   
                    shoot_cmd_send.shoot_single_flag=1;
                    shoot_start_flag=0;
                }
                    chassis_cmd_send.pump_mode = shoot_fetch_data.shoot_state;/* code */
                }
                if(chassis_cmd_send.pump_mode==1)
                {
                    shoot_stop_num++;
                    if(shoot_stop_num>=200)
                    {
                        shoot_stop_num=0;
                        shoot_cmd_send.shoot_single_flag=0; 
                    }
                }
                if (rc_data[TEMP].mouse.press_m&&shoot_flag==1)
                {
                    shoot_flag=0;
                     shoot_num++;      /* code */
                }
                else if (!(rc_data[TEMP].mouse.press_m))
                {
                    shoot_flag=1;
                }
                if ( shoot_num%2==0)
                {
                    shoot_cmd_send.shoot_mode = SHOOT_ON;
                         shoot_cmd_send.friction_mode =FRICTION_OFF;  /* code */
                     chassis_cmd_send.pump_mode = 0;
                }
                else if ( shoot_num%2==1)
                {
                    shoot_cmd_send.friction_mode =FRICTION_ON;
                    shoot_cmd_send.shoot_mode = SHOOT_ON;
                      /* code */
                }
              
                
        
                
        // else if (chassis_cmd_send.chassis_mode == CHASSIS_MINING)
        // {
        //     chassis_cmd_send.vx = 5000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].a - (float)rc_data[TEMP].key[KEY_PRESS].d); // _水平方向
        //     chassis_cmd_send.vy = 5000.0f * ((float)rc_data[TEMP].key[KEY_PRESS].w - (float)rc_data[TEMP].key[KEY_PRESS].s); // |竖直方向
        //     chassis_cmd_send.wz = 45.0f * ((float)rc_data[TEMP].key[KEY_PRESS].q - (float)rc_data[TEMP].key[KEY_PRESS].e);   // ↺自旋
        // }
    // }
    // else
    // {
    //     chassis_cmd_send.vx = 0.0f;
    //     chassis_cmd_send.vy = 0.0f;
    //     chassis_cmd_send.wz = 0.0f;
    // }
}
                if(rc_data[TEMP].key[KEY_PRESS].z)
                {
                    Flag_z=1;
                   
                }
                else if (Flag_z==1&&(!rc_data[TEMP].key[KEY_PRESS].z))
                {
                   shoot_p=shoot_p+0.01 ; /* code */
                   Flag_z=0;
                }
                

                if(rc_data[TEMP].key[KEY_PRESS].x)
                {
                   Flag_x=1;
                }
                else if (Flag_x==1&&(!rc_data[TEMP].key[KEY_PRESS].x))
                {
                    shoot_p=shoot_p-0.01 ;/* code */
                     Flag_x=0; 
                }
                shoot_cmd_send.shoot_num=shoot_p;
                if (rc_data[TEMP].key[KEY_PRESS].q&&shoot_change_flag==0)
                {
                    shoot_change++;
                    shoot_change_flag=1;
                    shoot_p=1.00;
                     /* code */
                }
                else if(!(rc_data[TEMP].key[KEY_PRESS].q))
                {
                    shoot_change_flag=0;   
                }
                if (shoot_change%2==0)
                {
                   // shoot_p=1.00;
                    shoot_cmd_send.bullet_speed=SMALL_AMU_15; /* code */
                     
                }
                else if (shoot_change%2==1)
                {
                    //shoot_p=1.0;
                    shoot_cmd_send.bullet_speed=SMALL_AMU_30; /* code */
                }
  //shoot_last_mode=
    // if (rc_data[TEMP].key[KEY_PRESS].z && rc_data[TEMP].key[KEY_PRESS].shift)
    //       {  chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_RESET;
    // }
    // else if (rc_data[TEMP].key[KEY_PRESS].z)
    //        { chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    //        }
    //        else if (rc_data[TEMP].key[KEY_PRESS].x)
    //        {
    //            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    //            gimbal_cmd_send.gimbal_mode = GIMBAL_NOMOVE;
    //        }
    // // 机械臂键鼠控制
    // // shift + //  W/S  //  Q/E  //  A/D
    // //         //  抬升 //  yaw1 //  yaw2
    // if ((rc_data[TEMP].key[KEY_PRESS].shift) && !(rc_data[TEMP].key[KEY_PRESS].ctrl))
    // {
    //     upper_cmd_send.joint_data.lift_dist += (1.2f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].w - 1.2f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s);
    //     upper_cmd_send.joint_data.yaw1 += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e);
    //     upper_cmd_send.joint_data.yaw2 += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d);
    // }

    // //  ctrl + //   W/S  //   A/D  //   Q/E
    // //           小pitch //  yaw3  //   roll
    // if ((!rc_data[TEMP].key[KEY_PRESS].shift) && (rc_data[TEMP].key[KEY_PRESS].ctrl))
    // {
    //     upper_cmd_send.joint_data.roll_differ += (0.25f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].e - 0.25f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].q);
    //     upper_cmd_send.joint_data.pitch_differ += (0.50f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w - 0.50f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s);
    //     upper_cmd_send.joint_data.yaw3 += (0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a - 0.5f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d);
    // }
    // if ((gimbal_cmd_send.gimbal_mode == GIMBAL_FIX_ANGLE_MODE) || (gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE) || (gimbal_cmd_send.gimbal_mode == GIMBAL_RESET)) // 这里保证复位模式只会被发送一次
    // {
    //     if (rc_data[TEMP].key_count[KEY_PRESS][15] % 2 == 0) // 最后一个键为b键
    //     {
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;
    //     }
    //     else
    //     {
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    //     }
    // }
    // 小云台键鼠控制
    //  ctrl + shift  //    W/S     //    A/D
    //                   云台pitch  //   云台yaw
    // if ((rc_data[TEMP].key[KEY_PRESS].shift) && (rc_data[TEMP].key[KEY_PRESS].ctrl))
    // {
    //     if (gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE)
    //     {
    //         gimbal_cmd_send.pitch_free_angle += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s);
    //         gimbal_fetch_data.yaw_free_angle_upload += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d);
    //     }
    //     else if (gimbal_cmd_send.gimbal_mode == GIMBAL_FIX_ANGLE_MODE)
    //     {
    //         gimbal_cmd_send.pitch_free_angle += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].w - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].s);
    //         gimbal_fetch_data.yaw_fixed_angle_upload += (1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].a - 1.0f * (float)rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].d);
    //     }
    // }

    // 小云台键鼠控制
    // gimbal_cmd_send.yaw_add_angle = -0.006f * rc_data[TEMP].mouse.x;
    // gimbal_cmd_send.pitch_add_angle = -0.006f * rc_data[TEMP].mouse.y;
    // if (rc_data->mouse.press_r)
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_RESET; // 图传复位

    // // ctrl + shift + r 刷新UI
    // if (rc_data[TEMP].key[KEY_PRESS].r && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].shift)
    //     chassis_cmd_send.UI_Init_Flag = 1;
    // // 这里由于优先级原因泵控制必须要在模式控制之前以强制覆盖
    // // shift + R 关闭真空泵
    // else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].r)
    //     chassis_cmd_send.pump_mode = VALVE_ALL_CLOSE;
    // // 单击 R 键打开真空泵
    // else if (rc_data[TEMP].key[KEY_PRESS].r)
    //     chassis_cmd_send.pump_mode |= VALVE_ALL_OPEN;
    // else
    //     chassis_cmd_send.UI_Init_Flag = 0;

    // /**************************************************   此处为动作组   **************************************************/
    // if (upper_cmd_send.upper_mode < UPPER_SLIVER_MINING)
    // {
    //     upper_cmd_send.upper_mode = UPPER_SINGLE_MOTOR;

    //     // ctrl + shift + F 键进入存矿仓矿石模式
    //     if (rc_data[TEMP].key[KEY_PRESS].f && rc_data[TEMP].key[KEY_PRESS].ctrl && rc_data[TEMP].key[KEY_PRESS].shift)
    //     {
    //         if (chassis_cmd_send.Ore_Storage_Flag2 == 1) // 先存2
    //         {
    //             upper_cmd_send.upper_mode = UPPER_STORAGE_ORE_1;
    //         }
    //         else
    //         {
    //             upper_cmd_send.upper_mode = UPPER_STORAGE_ORE_2;
    //         }
    //     }
    //     // shift + F 键进入取矿仓矿石模式
    //     else if (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].f)
    //     {
    //         if (chassis_cmd_send.Ore_Storage_Flag1 != 0) // 先取1
    //         {
    //             upper_cmd_send.upper_mode = UPPER_FETCH_ORE_1;
    //         }
    //         else
    //         {
    //             upper_cmd_send.upper_mode = UPPER_FETCH_ORE_2;
    //         }
    //     }
    //     // 单击 F 键进入小资源岛一位双矿模式
    //     else if (rc_data[TEMP].key[KEY_PRESS].f)
    //     {
    //         upper_cmd_send.upper_mode = UPPER_TWO_SLIVER_MINING;
    //     }

    //     // ctrl + shift + G 键重置机械臂状态
    //     if (rc_data[TEMP].key[KEY_PRESS].g && rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl)
    //     {
    //         upper_cmd_send.joint_data.yaw1 = 0;
    //         upper_cmd_send.joint_data.yaw2 = 0;
    //         upper_cmd_send.joint_data.yaw3 = 0;
    //         upper_cmd_send.joint_data.roll_differ = 0;
    //         upper_cmd_send.joint_data.pitch_differ = 0;
    //         upper_cmd_send.joint_data.lift_dist = LIFT_SAFE_HEIGHT;
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //     } // 单击 G 键进入大资源岛取矿模式
    //     else if (rc_data[TEMP].key[KEY_PRESS].g)
    //     {
    //         upper_cmd_send.upper_mode = UPPER_GLOD_MINING;
    //     }

    //     // ctrl + shift + c 进入校准模式
    //     // if (rc_data[TEMP].key[KEY_PRESS].c && rc_data[TEMP].key[KEY_PRESS].shift && rc_data[TEMP].key[KEY_PRESS].ctrl)
    //     //     upper_cmd_send.upper_mode = UPPER_CALI;
    //     // 单击 C 键进入取单银矿，地面矿模式
    //     if (rc_data[TEMP].key[KEY_PRESS].c)
    //     {
    //         upper_cmd_send.upper_mode = UPPER_SLIVER_MINING;
    //     }

    //     // 单击 V 键进入自定义控制器兑矿模式
    //     if (rc_data[TEMP].key[KEY_PRESS].v)
    //     {
    //         upper_cmd_send.upper_mode = UPPER_EXCHANGE;
    //     }
    // }

    // /**************************************************   此处为模式控制任务   **************************************************/

    // if (upper_cmd_send.upper_mode == UPPER_SLIVER_MINING) // 单银矿石，地矿
    // {
    //     if (upper_fetch_data.action_step == 1) // 初始化
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_MINING;
    //         chassis_cmd_send.pump_mode = VALVE_ARM1;
    //     }

    //     if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
    //         upper_cmd_send.stop_flag = 1;
    //     else if (upper_fetch_data.action_step == 2)
    //     {
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_SLIVER_MINGING_MODE; // 图传对准银矿
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].c && upper_fetch_data.action_step == 3) // 再次单击 F 键继续执行
    //     {
    //         upper_cmd_send.cfm_flag = 1;
    //     }

    //     // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
    //     if (upper_fetch_data.action_step != 3)
    //         upper_cmd_send.cfm_flag = 0;

    //     // 任务执行结束
    //     if (upper_fetch_data.action_step == 0)
    //     {
    //         upper_cmd_send.stop_flag = 0;
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         // chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;  // 在存放矿石后再修改底盘模式，用于微调车身状态
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传置位
    //         CmdRecvUpdate();
    //     }
    // }
    // else if (upper_cmd_send.upper_mode == UPPER_TWO_SLIVER_MINING) // 一位双矿
    // {
    //     if (upper_fetch_data.action_step == 1) // 初始化
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_MINING;
    //     }

    //     if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
    //         upper_cmd_send.stop_flag = 1;
    //     else if (upper_fetch_data.action_step == 2) // 先对准银矿
    //     {
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_TWO_SLIVER_MINGING_MODE1; // 调整图传以对准银矿
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 3) // 再次单击 F 键继续执行
    //     {
    //         upper_cmd_send.cfm_flag = 1;
    //         chassis_cmd_send.pump_mode = VALVE_ALL_OPEN;
    //     }
    //     else if (upper_fetch_data.action_step == 5) // 先对准矿仓
    //     {
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_TWO_SLIVER_MINGING_MODE2; // 调整图传以查看矿仓状态
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 6) // 再次单击 F 键继续执行
    //     {
    //         upper_cmd_send.cfm_flag = 2;
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 7) // 再次单击 F 键继续执行
    //     {
    //         chassis_cmd_send.pump_mode = VALVE_T_ALL_OPEN;       // 先关闭臂上气路再抬起
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传回正
    //         chassis_cmd_send.Ore_Storage_Flag1 = 1;
    //         chassis_cmd_send.Ore_Storage_Flag2 = 1;
    //         upper_cmd_send.cfm_flag = 3;
    //     }

    //     // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
    //     if (upper_fetch_data.action_step != 3 && upper_fetch_data.action_step != 6 && upper_fetch_data.action_step != 7)
    //         upper_cmd_send.cfm_flag = 0;

    //     // 任务执行结束
    //     if (upper_fetch_data.action_step == 0)
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         upper_cmd_send.stop_flag = 0;
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传置位
    //         CmdRecvUpdate();
    //     }
    // }
    // else if (upper_cmd_send.upper_mode == UPPER_FETCH_ORE_1) // 取矿仓1矿石
    // {
    //     if (upper_fetch_data.action_step == 1) // 初始化
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T_ALL_OPEN;
    //     }

    //     if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
    //         upper_cmd_send.stop_flag = 1;
    //     else if (upper_fetch_data.action_step == 2)
    //     {
    //         CmdRecvUpdate();                                      // 在单轴控制前先更新当前位姿
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FETCH_ORE_MODE1; // 对准矿仓
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 3) // 调整完毕位置后，再次单击 F 键继续执行
    //     {
    //         upper_cmd_send.cfm_flag = 1;
    //     }
    //     else if (upper_fetch_data.action_step == 4) // 关闭气泵
    //     {
    //         chassis_cmd_send.Ore_Storage_Flag1 = 0;
    //         chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T2;
    //         if (rc_data[TEMP].key[KEY_PRESS].f)
    //             upper_cmd_send.cfm_flag = 2;
    //     }
    //     else if (upper_fetch_data.action_step == 5)
    //     {
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传置位
    //     }

    //     // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
    //     if (upper_fetch_data.action_step != 3 && upper_fetch_data.action_step != 4)
    //         upper_cmd_send.cfm_flag = 0;

    //     // 任务执行结束
    //     if (upper_fetch_data.action_step == 0)
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         upper_cmd_send.stop_flag = 0;
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传置位
    //         CmdRecvUpdate();
    //     }
    // }
    // else if (upper_cmd_send.upper_mode == UPPER_FETCH_ORE_2) // 取矿仓2矿石
    // {
    //     if (upper_fetch_data.action_step == 1) // 初始化
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T2;
    //     }

    //     if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
    //         upper_cmd_send.stop_flag = 1;
    //     else if (upper_fetch_data.action_step == 2)
    //     {
    //         CmdRecvUpdate();                                      // 在单轴控制前先更新当前位姿
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FETCH_ORE_MODE2; // 对准矿仓
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 3) // 调整完毕位置后，再次单击 F 键继续执行
    //     {
    //         upper_cmd_send.cfm_flag = 1;
    //     }
    //     else if (upper_fetch_data.action_step == 4) // 关闭气泵
    //     {
    //         chassis_cmd_send.Ore_Storage_Flag2 = 0;
    //         chassis_cmd_send.pump_mode = VALVE_ARM1;
    //         if (rc_data[TEMP].key[KEY_PRESS].f)
    //             upper_cmd_send.cfm_flag = 2;
    //     }
    //     else if (upper_fetch_data.action_step == 5)
    //     {
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传置位
    //     }

    //     // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
    //     if (upper_fetch_data.action_step != 3 && upper_fetch_data.action_step != 4)
    //         upper_cmd_send.cfm_flag = 0;

    //     // 任务执行结束
    //     if (upper_fetch_data.action_step == 0)
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         upper_cmd_send.stop_flag = 0;
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传置位
    //         CmdRecvUpdate();
    //     }
    // }
    // else if (upper_cmd_send.upper_mode == UPPER_STORAGE_ORE_1) // 存矿仓1矿石
    // {
    //     if (upper_fetch_data.action_step == 1) // 初始化
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T_ALL_OPEN;
    //     }

    //     if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
    //         upper_cmd_send.stop_flag = 1;
    //     else if (upper_fetch_data.action_step == 3)
    //     {
    //         CmdRecvUpdate();                                        // 在单轴控制前先更新当前位姿
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_STORAGE_ORE_MODE1; // 对准矿仓
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 4) // 再次单击 F 键继续执行
    //     {
    //         upper_cmd_send.cfm_flag = 1;
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 5) // 再次单击 F 键继续执行
    //     {
    //         chassis_cmd_send.Ore_Storage_Flag1 = 1;
    //         chassis_cmd_send.pump_mode = VALVE_T_ALL_OPEN;
    //         upper_cmd_send.cfm_flag = 2;
    //     }

    //     // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
    //     if (upper_fetch_data.action_step != 4 && upper_fetch_data.action_step != 5)
    //         upper_cmd_send.cfm_flag = 0;

    //     // 任务执行结束
    //     if (upper_fetch_data.action_step == 0)
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         upper_cmd_send.stop_flag = 0;
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传置位
    //         CmdRecvUpdate();
    //     }
    // }
    // else if (upper_cmd_send.upper_mode == UPPER_STORAGE_ORE_2) // 存矿仓2矿石
    // {
    //     if (upper_fetch_data.action_step == 1) // 初始化
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         chassis_cmd_send.pump_mode = VALVE_ARM1 | VALVE_T2;
    //     }

    //     if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
    //         upper_cmd_send.stop_flag = 1;
    //     else if (upper_fetch_data.action_step == 3)
    //     {
    //         CmdRecvUpdate();                                        // 在单轴控制前先更新当前位姿
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_STORAGE_ORE_MODE2; // 对准矿仓
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 4) // 再次单击 F 键继续执行
    //     {
    //         upper_cmd_send.cfm_flag = 1;
    //     }
    //     else if (rc_data[TEMP].key[KEY_PRESS].f && upper_fetch_data.action_step == 5) // 再次单击 F 键继续执行
    //     {
    //         chassis_cmd_send.Ore_Storage_Flag2 = 1;
    //         chassis_cmd_send.pump_mode = VALVE_T2;
    //         upper_cmd_send.cfm_flag = 2;
    //     }

    //     // 该动作执行结束后再将flag置位，防止在多次循环中不能重复进入动作组判断
    //     if (upper_fetch_data.action_step != 4 && upper_fetch_data.action_step != 5)
    //         upper_cmd_send.cfm_flag = 0;

    //     // 任务执行结束
    //     if (upper_fetch_data.action_step == 0)
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         upper_cmd_send.stop_flag = 0;
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传置位
    //       CmdRecvUpdate();
    //     }
    // }
    // else if (upper_cmd_send.upper_mode == UPPER_GLOD_MINING) // 取金矿模式
    // {
    //     if (upper_fetch_data.action_step == 1) // 初始化
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_MINING;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_GOLD_MINING_MODE;
    //         chassis_cmd_send.pump_mode = VALVE_ARM1;
    //     }

    //     // 任务执行结束
    //     if (upper_fetch_data.action_step == 0)
    //     {
    //         // chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;  // 在存放矿石后再修改底盘模式，用于微调车身状态
    //         upper_cmd_send.stop_flag = 0;
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE; // 图传置位
    //         CmdRecvUpdate();
    //     }
    // }
    // else if (upper_cmd_send.upper_mode == UPPER_EXCHANGE) // 控制器兑换
    // {
    //     chassis_cmd_send.chassis_mode = CHASSIS_MINING;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;
    //     upper_cmd_send.ctrl_data.lift_dist = upper_cmd_send.joint_data.lift_dist;
    //     upper_cmd_send.ctrl_data.yaw1 = self_ctrl_data->yaw1;
    //     upper_cmd_send.ctrl_data.yaw2 = self_ctrl_data->yaw2;
    //     upper_cmd_send.ctrl_data.yaw3 = self_ctrl_data->yaw3;
    //     upper_cmd_send.ctrl_data.pitch = self_ctrl_data->pitch;
    //     upper_cmd_send.ctrl_data.roll = self_ctrl_data->roll;

    //     upper_cmd_send.joint_data.yaw1 = self_ctrl_data->yaw1; // 为 gimbal 更新当前的 yaw1 轴角度

    //     if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].x) // 退出模式
    //     {
    //         chassis_cmd_send.chassis_mode = CHASSIS_NORMAL;
    //         gimbal_cmd_send.gimbal_mode = GIMBAL_FIX_ANGLE_MODE;
    //         upper_cmd_send.upper_mode = UPPER_NO_MOVE;
    //         CmdRecvUpdate();
    //     }
    // }
    // if (chassis_cmd_send.Ore_Storage_Flag1) // 锁定矿仓气路防止矿石掉落
    // {
    //     chassis_cmd_send.pump_mode |= VALVE_T1;
    // }
    // if (chassis_cmd_send.Ore_Storage_Flag2)
    // {
    //     chassis_cmd_send.pump_mode |= VALVE_T2;
    // }
    // chassis_cmd_send.pump_mode = VALVE_ALL_CLOSE;  // 调试代码，关闭所有气路
#endif // USE_FS
}


/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨杆向下拨进入急停模式
#ifdef USE_DT7
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 还需添加重要应用和模块离线的判断
#endif
#ifdef USE_VT13
        if (switch_is_C(rc_data[TEMP].rc.switch_mid)) // 还需添加重要应用和模块离线的判断
#endif
#ifdef USE_FS
            if (switch_is_down(rc_data[TEMP].rc.switch_c))
#endif
            {
                robot_state = ROBOT_STOP;
                gimbal_cmd_send.gimbal_mode = GIMBAL_NOMOVE;
                chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
                shoot_cmd_send.shoot_mode = SHOOT_OFF;
                shoot_cmd_send.friction_mode = FRICTION_OFF;
                shoot_cmd_send.load_mode = LOAD_STOP;
                // upper_cmd_send.upper_mode = UPPER_ZERO_FORCE;
                LOGERROR("[CMD] emergency stop!");
            }
    // 遥控器右侧开关为[上],恢复正常运行
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD  
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
#ifdef USE_DT7
   // if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_mid(rc_data[TEMP].rc.switch_right))
#endif

#ifdef USE_VT13
        if (rc_data[TEMP].rc.Custom_button_right_count % 2 == 1) // 自定义按键右按下奇数次为键鼠控制
       
#endif

#ifdef USE_FS
           // if (0)
#endif
                MouseKeySet(); // 键鼠控制
             else
                RemoteControlSet(); // 遥控器控制

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // UpperJointConstrain(&upper_cmd_send.joint_data);

    GimbalSendUpdate();

    gimbal_cmd_send.shoot_flag = shoot_fetch_data.shoot_flag;
   //ChassisSendUpdate();
    
    
    
   
    // if (upper_last_mode != upper_cmd_send.upper_mode)
    // {
    //     CmdRecvUpdate();
    // }
    // upper_last_mode = upper_cmd_send.upper_mode;
    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
