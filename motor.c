#include "motor.h"
#include "PID.h"
#include "ch.h"
#include "dbus.h"
#include "hal.h"


int16_t check;

CANTxFrame txmsg;
CANRxFrame rxmsg;
int16_t motorSpeed[4];
int16_t result[4] = {0, 0, 0, 0};

float const maxSpeed = 4000;
float ki = 0.3;
float kp = 10;
float kd = 25;
static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) | CAN_BTR_TS1(8) | CAN_BTR_BRP(2)};

static THD_WORKING_AREA(can_rx_thd_wa, 512);
static THD_FUNCTION(can_rx_thd, p)
{
    (void)p;
    while (true)
    {
        if (canReceiveTimeout(
                   &CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_MS2I(1)) == MSG_OK)
        {
            // receiving rpm
            if (rxmsg.SID == 0x201)
            {
                motorSpeed[0] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                // rxcnt[0]++;
                // motorSpeed[0] = (motorSpeed[0]);
            }
            if (rxmsg.SID == 0x202)
            {
                motorSpeed[1] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                // rxcnt[1]++;
                motorSpeed[1] = (motorSpeed[1]) * 19 /27;
            }
            if (rxmsg.SID == 0x203)
            {
                motorSpeed[2] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                // rxcnt[2]++;
                // motorSpeed[2] = (motorSpeed[2])/19;
            }
            if (rxmsg.SID == 0x204)
            {
                motorSpeed[3] = rxmsg.data8[2] << 8 | rxmsg.data8[3];
                // rxcnt[3]++;
                // motorSpeed[3] = (motorSpeed[3])/19;
            }
        }
    }
}
void setSpeed(int i, float target)
{
    result[i] = PIDSet(&pidWheel[i], motorSpeed[i], target);
    txmsg.data8[i * 2] = (int)result[i] >> 8;
    txmsg.data8[i * 2 + 1] = (int)result[i] & 0xFF;
}

static THD_WORKING_AREA(can_tx_thd_wa, 256);
static THD_FUNCTION(can_tx_thd, p)
{
    (void)p;
    while (true)
    {
        txmsg.DLC = 8;
        txmsg.IDE = CAN_IDE_STD;
        txmsg.RTR = CAN_RTR_DATA;
        txmsg.SID = 0x200;
        // txmsg.data8[0] = (int)1000 >> 8;
        // txmsg.data8[1] = (int)1000 & 0xFF;
        movementControl(
            RCGet()->channel1 * maxSpeed/660, RCGet()->channel0*maxSpeed/660, RCGet()->channel2*maxSpeed/660);
        // setSpeed(0, 1000);
        // setSpeed(1, test);
        canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(1));
        chThdSleepMilliseconds(1);
    }
};


// speedX: X Direction SpeedY: Y Direction SpeedA: Angular Speed
void movementControl(float speedX, float speedY, float speedA)
{

    float speed0 = (speedX + speedY + speedA);
    float speed1 = (speedX - speedY - speedA);
    float speed2 = (speedX - speedY + speedA);
    float speed3 = (speedX + speedY - speedA);

    float max = speed0;
    if (max < speed1)
        max = speed1;
    if (max < speed2)
        max = speed2;
    if (max < speed3)
        max = speed3;

    if (max > maxSpeed)
    {
        speed0 = speed0 / max * maxSpeed;
        speed1 = speed1 / max * maxSpeed;
        speed2 = speed2 / max * maxSpeed;
        speed3 = speed3 / max * maxSpeed;
    }
    setSpeed(0, speed0);
    setSpeed(1, -speed1);
    setSpeed(2, speed2);
    setSpeed(3, -speed3);
}

float motorSpeedGet(int i)
{
    // chSysLock();
    float value = motorSpeed[i];
    // chSysUnlock();
    return value;
}

void motorInit(void)
{
    PIDInit(&pidWheel[0], maxSpeed, kp, ki, kd);
    PIDInit(&pidWheel[1], maxSpeed, kp, ki, kd);
    PIDInit(&pidWheel[2], maxSpeed, kp, ki, kd);
    PIDInit(&pidWheel[3], maxSpeed, kp, ki, kd);
    canStart(&CAND1, &cancfg);

    chThdCreateStatic(can_rx_thd_wa,
                      sizeof(can_rx_thd_wa),
                      NORMALPRIO + 15,
                      can_rx_thd,
                      NULL);
    chThdCreateStatic(can_tx_thd_wa,
                      sizeof(can_tx_thd_wa),
                      NORMALPRIO + 20,
                      can_tx_thd,
                      NULL);
}