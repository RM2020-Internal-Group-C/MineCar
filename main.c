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

// Calculation of BRP:
// Max APB1 clock Frequency: 36MHz
// Target Baudrate: 1MHz
// Nominal bit time: 12t_q

float check;
static volatile int cnt = 0;
static volatile int motor1Now = 0;
static volatile int motor1Last = 0;
//static const float RCToMotorRatio = 400 / 660;

static volatile int16_t result[4] = {0, 0, 0, 0};
//int16_t const maxSpeed = 300;

static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) | CAN_BTR_TS1(8) | CAN_BTR_BRP(2)};

static CANRxFrame rxmsg;
static CANTxFrame txmsg;
static volatile int16_t encoder[4];
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
    canStart(&CAND1, &cancfg);

    pwmStart(&PWMD3, &pwmcfg);
    pwmEnableChannel(&PWMD3, 0, 3);

    txmsg.DLC = 8;
    txmsg.IDE = CAN_IDE_STD;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.SID = 0x200;

    // Initialize dbus
    RCInit();

    // PID Initialize  wheelStruct; maxOutputCurrent; kp; ki; kd
    PIDInit(&pidmotor[0], 10000, 5, 0, 0);
    /***************************************************************
     ****************************四轮***********************************/

    while (true)
    {
        while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) ==
               MSG_OK)
        {
            // receiving rpm
            if (rxmsg.SID == 0x201)
            {
                encoder[0] = rxmsg.data8[0] << 8 | rxmsg.data8[1];
            }
            // if (rxmsg.SID == 0x202)
            // {
            //     encoder[1] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
            //     encoder[1] = (encoder[1]) / 19;
            // }
            // if (rxmsg.SID == 0x203)
            // {
            //     encoder[2] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
            //     encoder[2] = (encoder[2]) / 19;
            // }
            // if (rxmsg.SID == 0x204)
            // {
            //     encoder[3] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
            //     encoder[3] = (encoder[3]) / 19;
            // }

            // move
            //movementControl(RCGet()->channel3, RCGet()->channel2, RCGet()->channel0);
            // rc = RCGet()->channel2;
            motor1Now = encoder[0];
            result[0] = PIDSet(&pidmotor[0], 40000, motor1Last, motor1Now);
            txmsg.data8[0] = (int)result[0] >> 8;
            txmsg.data8[1] = (int)result[0] & 0xFF;
            motor1Last = motor1Now;


        
            cnt++;
            


            canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(1));
        }
        chThdSleepMilliseconds(1);
    }

}
