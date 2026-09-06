#include "shoot.h"
#include "robot_def.h"
#include "vofa.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "dm_motor.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r,*friction_l2,*friction_r2; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

static void ShootFeedbackUpdate();
 //static float shoot_speed_ff_target = 1000.0f;
void ShootInit()
{
  
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0, // 20
                .Ki = 0, // 1
                .Kd = 0,
                //.Kf =0.728,  
                //.Ref_FF=&shoot_speed_ff_target,  
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 30000,
                .MaxOut = 30000,
            },
            // .current_PID = {
            //     .Kp = 0.7, // 0.7
            //     .Ki = 0, // 0.1
            //     .Kd = 0.0001,
            //     .Improve = PID_Integral_Limit,
            //     .IntegralLimit = 10000,
            //     .MaxOut = 30000,
            // },
            // .smc = {
            //     .smc_e = 0,
            //     .smc_k = 0,
            //     .smc_delta =0,
            // },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag = SPEED_FEEDFORWARD,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 5,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 6; // 右前摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

       friction_config.can_init_config.tx_id = 8; // 左后摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_l2 = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 7; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r2 = DJIMotorInit(&friction_config);

    // 拨盘电机
    // Motor_Init_Config_s loader_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 3,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
    //             .Kp = 0, // 10
    //             .Ki = 0,
    //             .Kd = 0,
    //             .MaxOut = 200,
    //         },
    //         .speed_PID = {
    //             .Kp = 0, // 10
    //             .Ki = 0, // 1
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit,
    //             .IntegralLimit = 5000,
    //             .MaxOut = 5000,
    //         },
    //         .current_PID = {
    //             .Kp = 0, // 0.7
    //             .Ki = 0, // 0.1
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit,
    //             .IntegralLimit = 5000,
    //             .MaxOut = 5000,
    //         },
    //     },

    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
    //         .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
    //     },
    //     .motor_type = M3508 // 英雄使用m3508
    // };
   // loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}
static void ShootFeedbackUpdate()
{

    static uint8_t fric_count=0;//0 = 未检测到弹丸进入；1 = 检测到转速突降，弹丸正在经过摩擦轮
    static uint8_t shoot_once_flag=0;//1 = 待发射（就绪）；0 = 刚完成一次发射或不允许发射
    switch (shoot_cmd_recv.bullet_speed)
    {
    case SMALL_AMU_15:
    if ((-friction_l->measure.speed_aps +SHOOT_SPEED_MIN*shoot_cmd_recv.shoot_num)>3000)
    {
            fric_count=1;
    }   
    if((friction_l->measure.speed_aps <SHOOT_SPEED_MIN*shoot_cmd_recv.shoot_num)&&fric_count==1)
    {   
        shoot_feedback_data.shoot_state=SHOOT_OFF; // 发射完成
        shoot_once_flag=0;
        fric_count=0;
        shoot_feedback_data.shoot_flag=1;
    }

        break;

        case SMALL_AMU_30:
            if ((-friction_l->measure.speed_aps +SHOOT_SPEED_MAX*shoot_cmd_recv.shoot_num)>4000)
    {
            fric_count=1;
    }   
    if((friction_l->measure.speed_aps <SHOOT_SPEED_MAX*shoot_cmd_recv.shoot_num)&&fric_count==1)
    {   
        shoot_feedback_data.shoot_state=SHOOT_OFF; // 发射完成
        shoot_once_flag=0;
        fric_count=0; 
        shoot_feedback_data.shoot_flag=1;
    }
        break;
    default:
        break;
    }


     if (shoot_cmd_recv.shoot_single_flag==1&& shoot_once_flag==1)
    {
        shoot_feedback_data.shoot_state=SHOOT_ON; // 正在发射
        shoot_feedback_data.shoot_flag=0;
    }
     if(shoot_cmd_recv.shoot_single_flag==0)
    {
        shoot_feedback_data.shoot_state=SHOOT_OFF;
        shoot_once_flag=1;
         shoot_feedback_data.shoot_flag=0;
    }
    if (shoot_once_flag==0)
    {
       shoot_feedback_data.shoot_state=SHOOT_OFF;
        shoot_feedback_data.shoot_flag=0;
       
        /* code */
    }
    if (friction_l2->measure.speed_aps<9000)
    {
        shoot_feedback_data.shoot_state=SHOOT_OFF; /* code */
    }
    
    // 推送消息

}





/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    static float arr[5] = {0};
    static float set_speed = SHOOT_SPEED_MAX;
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    arr[0] = friction_l2->measure.speed_aps;
    arr[1] = friction_r2->measure.speed_aps;
    arr[2] = friction_l->measure.speed_aps;
    arr[3] = friction_r->measure.speed_aps;
    arr[4] = set_speed;
    //vofa_justfloat_output(arr, 5, &huart1);
     DJIMotorSetSMCdifferent(friction_l, friction_r->measure.speed_aps+friction_l->measure.speed_aps);
     DJIMotorSetSMCdifferent(friction_r, friction_r->measure.speed_aps+friction_l->measure.speed_aps);
     DJIMotorSetSMCdifferent(friction_l2,friction_r2->measure.speed_aps+friction_l2->measure.speed_aps);
     DJIMotorSetSMCdifferent(friction_r2, friction_r2->measure.speed_aps+friction_l2->measure.speed_aps);
    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(friction_l2);
        DJIMotorStop(friction_r2);
        // DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(friction_l2);
        DJIMotorEnable(friction_r2);
        // DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    // switch (shoot_cmd_recv.load_mode)
    // {
    // // 停止拨盘
    // case LOAD_STOP:
    //     //DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
    //     //DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
    //     break;
    // // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    // case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
    //     //DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
    //     //DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
    //     hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
    //     dead_time = 150;                                                                    // 完成1发弹丸发射的时间
    //     break;
    // // 三连发,如果不需要后续可能删除
    // case LOAD_3_BULLET:
    //    // DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
    //     //DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
    //     hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
    //     dead_time = 300;                                                                        // 完成3发弹丸发射的时间
    //     break;
    // // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    // case LOAD_BURSTFIRE:
    //     //DJIMotorOuterLoop(loader, SPEED_LOOP);

    //    // DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
    //     // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
    //     break;
    // // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // // 也有可能需要从switch-case中独立出来
    // case LOAD_REVERSE:
    //     //DJIMotorOuterLoop(loader, SPEED_LOOP);
    //     // ...
    //     break;
    // default:
    //     while (1)
    //         ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    // }

    // // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {

        ShootFeedbackUpdate();
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetSMCRef(friction_l, SHOOT_SPEED_MIN*shoot_cmd_recv.shoot_num);
            DJIMotorSetSMCRef(friction_r, -SHOOT_SPEED_MIN*shoot_cmd_recv.shoot_num);
            DJIMotorSetSMCRef(friction_l2, SHOOT_SPEED_MIN*0.6*shoot_cmd_recv.shoot_num);
            DJIMotorSetSMCRef(friction_r2, -SHOOT_SPEED_MIN*0.6*shoot_cmd_recv.shoot_num);

            // DJIMotorSetRef(friction_l, 25600);
            // DJIMotorSetRef(friction_r, 25600);
            // DJIMotorSetRef(friction_l2, 25000);
            // DJIMotorSetRef(friction_r2, 25000);
            break;
        case SMALL_AMU_18:
            DJIMotorSetSMCRef(friction_l, 0);
            DJIMotorSetSMCRef(friction_r, 0);
            DJIMotorSetSMCRef(friction_l2, 0);
            DJIMotorSetSMCRef(friction_r2, 0);
            break;
        case SMALL_AMU_30:
            DJIMotorSetSMCRef(friction_l, SHOOT_SPEED_MAX*shoot_cmd_recv.shoot_num);
            DJIMotorSetSMCRef(friction_r, -SHOOT_SPEED_MAX*shoot_cmd_recv.shoot_num);
            DJIMotorSetSMCRef(friction_l2, SHOOT_SPEED_MAX * 0.6*shoot_cmd_recv.shoot_num);
            DJIMotorSetSMCRef(friction_r2, -SHOOT_SPEED_MAX * 0.6*shoot_cmd_recv.shoot_num);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            DJIMotorSetRef(friction_l2, 0);
            DJIMotorSetRef(friction_r2, 0);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetSMCRef(friction_l, 0);
        DJIMotorSetSMCRef(friction_r, 0);
        DJIMotorSetSMCRef(friction_l2, 0);
        DJIMotorSetSMCRef(friction_r2, 0);
    }

    // // 开关弹舱盖
    // if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    // {
    //     //...
    // }
    // else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    // {
    //     //...
    // }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}