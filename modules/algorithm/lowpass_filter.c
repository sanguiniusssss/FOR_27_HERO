/**
 * @file lowpass_fliter.c
 * @author Weedy
 * @brief  离散型一阶低通滤波器定义
 * @version beta
 * @date 2025-04-24
 *
 * @example:
 * 
 */

#include "user_lib.h"
#include "lowpass_filter.h"

/**
 * @brief 低通滤波器的注册函数
 * @param cutoff_freq 截止频率 / Hz
 * @param sample_time 采样时间 / s
 * 
 * @retval ins 低通滤波器实例
 */
LowpassFilterInstance *LowpassFilterInit(LowpassFilterConfig *conf)
{
    LowpassFilterInstance *ins = (LowpassFilterInstance *)user_malloc(sizeof(LowpassFilterInstance));
    memset(ins, 0, sizeof(LowpassFilterInstance));

    float RC = 1.0f /(2 * PI * conf->cutoff_freq);                  // 计算时间常数
    ins->alpha = conf->sample_time /(RC + conf->sample_time);       // α = 1 - exp(-2 * π * fc * T)

    ins->Init_flag = 1;

    return ins;
}

/**
 * @brief 低通滤波器的数据更新函数
 * @param ins 低通滤波器实例
 * @param data 等待滤波的数据
 * 
 * @retval last_output 滤波器输出数据
 */
float LowpassFilterUpdate(LowpassFilterInstance *ins, float data)
{
    if(ins->Init_flag == 1)
    {
        ins->last_output = data;
        ins->Init_flag = 0;
    }
    ins->last_output = ins->alpha * data + (1 - ins->alpha)* ins->last_output;
    return ins->last_output;
}