#ifndef __MOTOR_H_
#define __MOTOR_H_

void motor_init(void);
void motor_set_A(unsigned char dir,unsigned short pwm_duty);
void motor_set_B(unsigned char dir,unsigned short pwm_duty);
void motor_stop(void);

#endif
