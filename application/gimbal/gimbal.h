#ifndef GIMBAL_H
#define GIMBAL_H
#include <stdint.h>
#define HALF_GYRO_RANGE 180
#define GYRO_RANGE 360
/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 *
 */
void GimbalTask();

/**
 * @brief 自瞄是否允许开火(目标在开火窗口内)
 * @return 1=允许, 0=不允许
 */
uint8_t GimbalAutoFireAllow(void);

#endif // GIMBAL_H