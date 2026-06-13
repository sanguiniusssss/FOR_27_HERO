#ifndef CHASSIS_H
#define CHASSIS_H



#define SUPERCA_V_MAX 24.0f // 超电压上限,单位:V
#define SUPERCA_V_MIN 10.0f // 超电压下限,单位:

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

#endif // CHASSIS_H