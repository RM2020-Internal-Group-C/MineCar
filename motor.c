#include "motor.h"
#include "ch.h"
#include "hal.h"
CANTxFrame txmsg;
CANRxFrame rxmsg;
int16_t motorSpeed[4];

static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) | CAN_BTR_TS1(8) | CAN_BTR_BRP(2)};

static THD_WORKING_AREA(can_thd_wa, 512);
static THD_FUNCTION(can_thd, p) {
    (void)p;
        txmsg.DLC = 8;
        txmsg.IDE = CAN_IDE_STD;
        txmsg.RTR = CAN_RTR_DATA;
        txmsg.SID = 0x200;
        while(true)
    while (canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) ==
               MSG_OK)
        {
            // receiving rpm
            if (rxmsg.SID == 0x201)
            {
                motorSpeed[0] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                motorSpeed[0] = (motorSpeed[0]) / 19;
            }
            if (rxmsg.SID == 0x202)
            {
                motorSpeed[1] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                motorSpeed[1] = (motorSpeed[1]) / 19;
            }
            if (rxmsg.SID == 0x203)
            {
                motorSpeed[2] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                motorSpeed[2] = (motorSpeed[2]) / 19;
            }
            if (rxmsg.SID == 0x204)
            {
                motorSpeed[3] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                motorSpeed[3] = (motorSpeed[3]) / 19;
            }
            canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(1));
        }
        chThdSleepMilliseconds(1);
        

}

void canSetSpeed(int i, int16_t target)
{
    
    txmsg.data8[i * 2] = target >> 8;
    txmsg.data8[i * 2 + 1] = target & 0xFF;
}

float motorSpeedGet(int i) {
    return motorSpeed[i];
}

void motorInit(void){

    canStart(&CAND1, &cancfg);

    chThdCreateStatic(can_thd_wa, sizeof(can_thd_wa), NORMALPRIO + 8, can_thd, NULL);
}