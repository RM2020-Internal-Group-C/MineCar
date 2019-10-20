/**
 * @file main.c
 * @brief Main file
 * @version 0.1
 * @date 2019-09-23
 *
 * @copyright Copyright (c) 2019
 *
 */
#include "ch.h"
#include "hal.h"
#include "PID.h"

// Calculation of BRP:
// Max APB1 clock Frequency: 36MHz
// Target Baudrate: 1MHz
// Nominal bit time: 12t_q

int setSpeed = 390;
float result;

static const CANConfig cancfg = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
  CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
};
static CANRxFrame rxmsg;
static CANTxFrame txmsg;
static volatile int16_t encoder[4] = {0,0,0,0};
static const PWMConfig pwmcfg = {1000000,
                                 10,
                                 NULL,
                                 {{PWM_OUTPUT_ACTIVE_HIGH, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL},
                                  {PWM_OUTPUT_DISABLED, NULL}},
                                 0,
                                 0};

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
    canStart(&CAND1, &cancfg);

    pwmStart(&PWMD3, &pwmcfg);
    pwmEnableChannel(&PWMD3, 0, 3);

    txmsg.DLC = 8;
    txmsg.IDE = CAN_IDE_STD;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.SID = 0x200;

    
    //PID Initialize
    PIDInit(&pidWheel[0], 5000, 420, 0.015, 10);
    /*
    PIDInit(pidWheel[1], 2000, 0.2, 0.015, 0.2);
    PIDInit(pidWheel[2], 2000, 0.2, 0.015, 0.2);
    PIDInit(pidWheel[3], 2000, 0.2, 0.015, 0.2);
    */
    // set current 1000 for the motor 1
    
    
    //txmsg.data8[0] = 1000 >> 8;
    //txmsg.data8[1] = 1000 & 0xFF;
    

    /***************************************************************
     ***************************************************************/

    while (true)
    {
        canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(1));
        while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
            if (rxmsg.SID == 0x201) {
                encoder[0] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                encoder[0] = -(encoder[0]/19);
           }
           /*
           if (rxmsg.SID == 0x202) {
                encoder[1] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
           }
           if (rxmsg.SID == 0x203) {
                encoder[2] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
           }
           if (rxmsg.SID == 0x204) {
                encoder[3] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
           }
           
           result = PIDSet(pidWheel[0], encoder[0], setSpeed);
           txmsg.data8[0] = (int)PIDSet(pidWheel[0], encoder[0], setSpeed)/2 >> 8;
           txmsg.data8[1] = (int)PIDSet(pidWheel[0], encoder[0], setSpeed)/2 & 0xFF;
           */
           
           
            result = PIDSet(&pidWheel[0], encoder[0], setSpeed);
            txmsg.data8[0] = (int)result/2 >> 8;
            txmsg.data8[1] = (int)result/2 & 0xFF;
            
            
            
        }   
        chThdSleepMilliseconds(2);
    }
}
