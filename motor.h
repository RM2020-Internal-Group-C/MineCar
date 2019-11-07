#ifndef __MOTOR_H_
#define __MOTOR_H_
#include "stdint.h"

#define MAX_SPEED       4000
#define KP              20
#define KI              0.015
#define KD              0.5
float motorSpeedGet(int i);
void motorInit(void);
void movementControl(float speedX, float speedY, float speedA);
#endif /* __MOTOR_H_ */

