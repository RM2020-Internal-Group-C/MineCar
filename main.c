/**
 * @file main.c
 * @brief Main file
 * @version 0.1
 * @date 2019-09-23
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "PID.h"
#include "ch.h"
#include "dbus.h"
#include "hal.h"
#include "motor.h"

// Calculation of BRP:
// Max APB1 clock Frequency: 36MHz
// Target Baudrate: 1MHz
// Nominal bit time: 12t_q

float check;

//static const float RCToMotorRatio = 400 / 660;

int16_t result[4] = {0, 0, 0, 0};
//int16_t const maxSpeed = 300;

static const PWMConfig pwmcfg = {1000000,
                                 10,
                                 NULL,
                                 {{PWM_OUTPUT_ACTIVE_HIGH, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL}},
                                 0,
                                 0};

// i stands for the index of motor, respectively 0, 1, 2, 3; targetSpeed
void setSpeed(int i, float target) {
    result[i] = PIDSet(&pidWheel[i], motorSpeedGet(i), target);
    canSetSpeed(i, (int16_t)result[i]);
}
// speedX: X Direction SpeedY: Y Direction SpeedA: Angular Speed
void movementControl(float speedX, float speedY, float speedA)
{
    float speed0 = speedX + speedY + speedA;
    float speed1 = speedX - speedY - speedA;
    float speed2 = speedX - speedY + speedA;
    float speed3 = speedX + speedY - speedA;

    float max = speed0;
    if (max < speed1)
        max = speed1;
    if (max < speed2)
        max = speed2;
    if (max < speed3)
        max = speed3;

    if (max > 400)
    {
        speed0 = speed0 / max * 400;
        speed1 = speed1 / max * 400;
        speed2 = speed2 / max * 400;
        speed3 = speed3 / max * 400;
    }
    setSpeed(0, speed0);
    setSpeed(1, -speed1);
    setSpeed(2, speed2);
    setSpeed(3, -speed3);
}

int main(void)
{
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();     
    // CAN REMAP
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;
    // DBUS REMAP
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

    pwmStart(&PWMD3, &pwmcfg);
    pwmEnableChannel(&PWMD3, 0, 3);

    motorInit();

    // Initialize dbus
    RCInit();

    // PID Initialize  wheelStruct; maxOutputCurrent; kp; ki; kd
    PIDInit(&pidWheel[0], 2000, 5, 0, 0);
    PIDInit(&pidWheel[1], 2000, 5, 0, 0);
    PIDInit(&pidWheel[2], 2000, 5, 0, 0);
    PIDInit(&pidWheel[3], 2000, 5, 0, 0);

    /***************************************************************
     ****************************四轮***********************************/

    while (true)
    {
        movementControl(RCGet()->channel3, RCGet()->channel2, RCGet()->channel0);
 
        chThdSleepMilliseconds(2);
    }
}
