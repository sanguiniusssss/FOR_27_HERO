#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
void GimbalInit(void);

/**
 * @brief 云台任务,由RobotTask调用 (200Hz)
 *
 */
void GimbalTask(void);

#endif // GIMBAL_H
