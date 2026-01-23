#ifndef __PID_H
#define __PID_H

typedef struct {
    volatile float Target;    // 异步修改的变量必须加
    volatile float Actual;    // 传感器/中断更新实际值
    volatile float Out;       // PID输出值
    
    float Kp;
    float Ki;
    float Kd;
    
    volatile float Error0;    // 当前误差
    volatile float Error1;    // 上一次误差
    volatile float ErrorInt;  // 误差积分
    
    float OutMax;
    float OutMin;
} PID_t;

void PID_Update(PID_t *p);

#endif
