#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#define GIMBAL_YAW_ECD 6060.0f
#define GIMBAL_PITCH_ECD 6060.0f

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H