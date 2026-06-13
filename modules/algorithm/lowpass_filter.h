#ifndef _LOWPASS_FILTER_H_

#define _LOWPASS_FILTER_H_

typedef struct 
{
    float alpha;        // 滤波器常数
    float last_output;  // 输出值，同时作为上次输出值

    float Init_flag;
} LowpassFilterInstance;

typedef struct 
{
    float cutoff_freq;  // 截止频率 / Hz
    float sample_time;  // 采样时间 / s
} LowpassFilterConfig;


LowpassFilterInstance *LowpassFilterInit(LowpassFilterConfig *conf);

float LowpassFilterUpdate(LowpassFilterInstance *ins, float data);

#endif // _LOWPASS_FILTER_H_