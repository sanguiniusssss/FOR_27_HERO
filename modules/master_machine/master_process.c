/**
 * @file master_process.c
 * @brief  与视觉上位机(NUC)的通信模块, TJ 协议, 走 USB 虚拟串口
 */
#include "master_process.h"
#include "crc_ref.h"
#include "daemon.h"
#include "bsp_log.h"
#include "ins_task.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static uint8_t *vis_recv_buff; // USB 接收缓冲, 由 USBInit 提供

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief 离线回调, USB 通信无需重启服务, 仅记录
 */
static void VisionOfflineCallback(void *id)
{
    (void)id;
    LOGWARNING("[vision] vision offline");
}

/**
 * @brief USB 接收回调: 校验 CRC16 与帧头, 解析 NUC 下发的目标角
 */
static void DecodeVision(uint16_t recv_len)
{
    if (recv_len != sizeof(TJ_R_t))
        return;
    if (Verify_CRC16_Check_Sum(vis_recv_buff, sizeof(TJ_R_t)) != TRUE)
        return;

    TJ_R_t *frame = (TJ_R_t *)vis_recv_buff;
    if (frame->head[0] != 'S' || frame->head[1] != 'P')
        return;

    recv_data.state = frame->mode;
    recv_data.yaw = frame->yaw;
    recv_data.yaw_vel = frame->yaw_vel;
    recv_data.pitch = frame->pitch;
    recv_data.pitch_vel = frame->pitch_vel;

    DaemonReload(vision_daemon_instance); // 喂狗
}

Vision_Recv_s *VisionInit(void)
{
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5, // 50ms 未收到数据判定掉线
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

Vision_Recv_s *VisionGetRecvData(void)
{
    return &recv_data;
}

/**
 * @brief 组 TJ_T 帧并发送给 NUC
 */
void VisionSend(void)
{
    static TJ_T_t tj_t; // 必须为 static, USB 发送完成前不能被释放
    attitude_t *ins = INS_Init();
    float *q = get_INS_quat_point();

    tj_t.head[0] = 'S';
    tj_t.head[1] = 'P';
    tj_t.mode = 1;
    tj_t.q[0] = q[0];
    tj_t.q[1] = q[1];
    tj_t.q[2] = q[2];
    tj_t.q[3] = q[3];
    tj_t.yaw = send_data.yaw;
    tj_t.pitch = send_data.pitch;
    tj_t.yaw_vel = ins->Gyro[2];   // yaw 角速度
    tj_t.pitch_vel = ins->Gyro[0]; // pitch 角速度
    tj_t.bullet_speed = (float)send_data.bullet_speed;
    tj_t.bullet_count = 0;
    tj_t.crc16 = Get_CRC16_Check_Sum((uint8_t *)&tj_t, sizeof(TJ_T_t) - 2, 0xffff);

    USBTransmit((uint8_t *)&tj_t, sizeof(TJ_T_t));
}
