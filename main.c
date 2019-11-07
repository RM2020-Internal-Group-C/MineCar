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

float rccheck;
float PIDcheck2;
float PIDcheck3;

//static const float RCToMotorRatio = 400 / 660;

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

// i stands for the index of motor, respectively 0, 1, 2, 3; targetSpee

void pneumaticOpen5()
{
    palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
    palWritePad(GPIOA, 8, PAL_HIGH);
}

void pneumaticClose5()
{
    palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
    palWritePad(GPIOA, 8, PAL_LOW);
}

void pneumaticOpen4()
{
    palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL);
    palWritePad(GPIOA, 9, PAL_HIGH);
}

void pneumaticClose4()
{
    palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL);
    palWritePad(GPIOA, 9, PAL_LOW);
}

void pneumaticOpen3()
{
    palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL);
    palWritePad(GPIOA, 9, PAL_HIGH);
}

void pneumaticClose3()
{
    palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL);
    palWritePad(GPIOA, 9, PAL_LOW);
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

    /***************************************************************
     ****************************四轮***********************************/

    while (true)
    {
        //movementControl(RCGet()->channel3, RCGet()->channel2, RCGet()->channel0);
        palSetLine(LINE_LED);
        rccheck = RCGet()->channel3*400/660;
        // PIDcheck1 = pidWheel[1].errNOW;
        // PIDcheck2 = pidWheel[1].set;
        // PIDcheck3 = pidWheel[1].get;
        chThdSleepMilliseconds(5);
        palClearLine(LINE_LED);
    }
}
