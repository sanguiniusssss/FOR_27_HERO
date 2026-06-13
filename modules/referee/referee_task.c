/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 * @note
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
    // referee_recv_info->referee_id.Robot_Color = Robot_Blue;
    // referee_recv_info->referee_id.Robot_ID = 2;
    // referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    // referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag =1;
    return referee_recv_info;
}

void UITask()
{
    // RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static String_Data_t UI_State_sta[6]; // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6]; // 机器人状态,动态先add才能change

static Graph_Data_t car_line[2];
void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制车辆状态标志指示
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 150, 750, "Flag1:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "Flag2:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "chassis:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 600, "upper:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 550, "gimbal:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "0");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "0");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "ZERO_FORCE");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, "ZERO_FORCE       ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, "FIX_ANGLE    ");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    UILineDraw(&car_line[0], "car0", UI_Graph_ADD, 8, UI_Color_Yellow, 4, 801, 278, 599, 0);//车身线
    UILineDraw(&car_line[1], "car1", UI_Graph_ADD, 8, UI_Color_Yellow, 4, 1149, 277, 1356, 0);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, car_line[0], car_line[1]);}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
// static uint8_t count = 0;
// static uint16_t count1 = 0;
// static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
// {
//     count++;
//     if (count >= 50)
//     {
//         count = 0;
//         count1++;
//     }
//     switch (count1 % 10)
//     {
//     case 0:
//     {
//         _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
//         _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
//         _Interactive_data->upper_mode = UPPER_ZERO_FORCE;
//         _Interactive_data->flag1_mode = 0;
//         _Interactive_data->flag2_mode = 0;
//         break;
//     }
//     case 1:
//     {
//         _Interactive_data->chassis_mode = CHASSIS_NORMAL;
//         _Interactive_data->gimbal_mode = GIMBAL_FIX_ANGLE_MODE;
//         _Interactive_data->upper_mode = UPPER_NO_MOVE;
//         break;
//     }
//     case 2:
//     {
//         _Interactive_data->chassis_mode = CHASSIS_NO_MOVE;
//         _Interactive_data->gimbal_mode = GIMBAL_SLIVER_MINGING_MODE;
//         _Interactive_data->upper_mode = UPPER_SINGLE_MOTOR;
//         _Interactive_data->flag1_mode = 1;
//         _Interactive_data->flag2_mode = 1;
//         break;
//     }
//     case 3:
//     {
//         _Interactive_data->chassis_mode = CHASSIS_MINING;
//         _Interactive_data->gimbal_mode = GIMBAL_TWO_SLIVER_MINGING_MODE1;
//         _Interactive_data->upper_mode = UPPER_SLIVER_MINING;
//         break;
//     }
//     default:
//         break;
//     }
// }

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
        case CHASSIS_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, "ZERO_FORCE");
            break;
        case CHASSIS_NORMAL:
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, "NORMAL    ");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        case CHASSIS_NO_MOVE:
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, "NO_MOVE   ");
            break;
        case CHASSIS_MINING:
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, "MINING    ");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // gimbal
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        switch (_Interactive_data->gimbal_mode)
        {
        case GIMBAL_FREE_MODE:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "FREE         ");
            break;
        }
        case GIMBAL_FIX_ANGLE_MODE:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "FIX_ANGLE    ");
            break;
        }
        case GIMBAL_SLIVER_MINGING_MODE:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "SLIVER       ");
            break;
        }
        case GIMBAL_TWO_SLIVER_MINGING_MODE1:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "TWO_SLIVER_1 ");
            break;
        }
        case GIMBAL_TWO_SLIVER_MINGING_MODE2:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "TWO_SLIVER_2 ");
            break;
        }
        case GIMBAL_STORAGE_ORE_MODE1:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "STORAGE_ORE_1");
            break;
        }
        case GIMBAL_STORAGE_ORE_MODE2:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "STORAGE_ORE_2");
            break;
        }
        case GIMBAL_FETCH_ORE_MODE1:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "FETCH_ORE_1  ");
            break;
        }
        case GIMBAL_FETCH_ORE_MODE2:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "FETCH_ORE_2  ");
            break;
        }
        case GIMBAL_GOLD_MINING_MODE:
        {
            UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, "GOLD_MINING  ");
            break;
        }
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }
    if (_Interactive_data->Referee_Interactive_Flag.upper_flag == 1)
    {
        switch (_Interactive_data->upper_mode)
        {
        case UPPER_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "ZERO_FORCE       ");
            break;
        case UPPER_NO_MOVE:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "NO_MOVE          ");
            break;
        case UPPER_CALI:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "CALI             ");
            break;
        case UPPER_SINGLE_MOTOR:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "SINGLE_MOTOR     ");
            break;
        case UPPER_SLIVER_MINING:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "SLIVER_MINING    ");
            break;
        case UPPER_TWO_SLIVER_MINING:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "TWO_SLIVER_MINING");
            break;
        case UPPER_FETCH_ORE_1:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "FETCH_ORE_1      ");
            break;
        case UPPER_FETCH_ORE_2:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "FETCH_ORE_2      ");
            break;
        case UPPER_GLOD_MINING:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "GLOD_MINING      ");
            break;
        case UPPER_STORAGE_ORE_1:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "STORAGE_ORE_1    ");
            break;
        case UPPER_STORAGE_ORE_2:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "STORAGE_ORE_2    ");
            break;
        case UPPER_EXCHANGE:
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, "EXCHANGE         ");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.upper_flag = 0;
    }
    if (_Interactive_data->Referee_Interactive_Flag.flag1_flag == 1)
    {
        switch (_Interactive_data->flag1_mode)
        {
        case 0:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "0");
            break;
        case 1:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "1");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.flag1_flag = 0;
    }
    if (_Interactive_data->Referee_Interactive_Flag.flag2_flag == 1)
    {
        switch (_Interactive_data->flag2_mode)
        {
        case 0:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "0");
            break;
        case 1:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "1");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.flag2_flag = 0;
    }
    // shoot
    // if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    //     _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    // }
    // friction
    // if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    //     _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    // }
    // lid
    // if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, _Interactive_data->lid_mode == LID_OPEN ? "open " : "close");
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    //     _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    // }
    // power
    // if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    // {
    //     UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
    //     UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, (uint32_t)750 + _Interactive_data->Chassis_Power_Data.chassis_power_mx * 30, 160);
    //     UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
    //     _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    // }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }
    if (_Interactive_data->upper_mode != _Interactive_data->upper_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.upper_flag = 1;
        _Interactive_data->upper_last_mode = _Interactive_data->upper_mode;
    }
    if (_Interactive_data->flag1_mode != _Interactive_data->flag1_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.flag1_flag = 1;
        _Interactive_data->flag1_last_mode = _Interactive_data->flag1_mode;
    }
    if (_Interactive_data->flag2_mode != _Interactive_data->flag2_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.flag2_flag = 1;
        _Interactive_data->flag1_last_mode = _Interactive_data->flag2_mode;
    }
    // if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
    //     _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    // }

    // if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
    //     _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    // }

    // if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
    //     _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    // }

    // if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
    //     _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    // }
}
