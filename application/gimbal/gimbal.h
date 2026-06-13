#ifndef GIMBAL_H
#define GIMBAL_H
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

#endif // GIMBAL_H