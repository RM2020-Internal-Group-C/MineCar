#ifndef __MOTOR_H_
#define __MOTOR_H_
#include "stdint.h"
float motorSpeedGet(int i);
void motorInit(void);
void canSetSpeed(int i,int16_t target);
#endif /* __MOTOR_H_ */

