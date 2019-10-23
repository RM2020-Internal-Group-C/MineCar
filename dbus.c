#include "dbus.h"
#include "ch.h"
#include "hal.h"

static uint8_t rxbuf[DBUS_BUFFER_SIZE];
static RC_control_t rcCtrl;
static thread_reference_t uart_dbus_thread_handler = NULL;
systime_t updateTime;

static void rxend(UARTDriver *uartp)
{
    chSysLockFromISR();
    chThdResumeI(&uart_dbus_thread_handler, MSG_OK);
    chSysUnlockFromISR();
}
static UARTConfig uartcfg = {
    NULL, NULL, rxend, NULL, NULL, 100000, USART_CR1_PCE, 0, 0};

static void RCReset(void)
{
    rcCtrl.channel0 = 0;
    rcCtrl.channel1 = 0;
    rcCtrl.channel2 = 0;
    rcCtrl.channel3 = 0;
    rcCtrl.s1 = 0;
    rcCtrl.s2 = 0;
}

static void processRxData(void)
{
    rcCtrl.channel0 =   ((int16_t)rxbuf[0] | ((int16_t)rxbuf[1] << 8)) & 0x07FF;
    rcCtrl.channel0 -=  RC_CH_VALUE_OFFSET;
    rcCtrl.channel1 =   (((int16_t)rxbuf[1] >> 3) | ((int16_t)rxbuf[2] << 5)) & 0x07FF;
    rcCtrl.channel1 -=  RC_CH_VALUE_OFFSET;
    rcCtrl.channel2 =   (((int16_t)rxbuf[2] >> 6) | ((int16_t)rxbuf[3] << 2) | ((int16_t)rxbuf[4] << 10)) & 0x07FF;
    rcCtrl.channel2 -=  RC_CH_VALUE_OFFSET;
    rcCtrl.channel3 =   (((int16_t)rxbuf[4] >> 1) | ((int16_t)rxbuf[5] << 7)) & 0x07FF;
    rcCtrl.channel3 -=  RC_CH_VALUE_OFFSET;

    rcCtrl.s1 = ((rxbuf[5] >> 4) & 0x000C) >> 2;
    rcCtrl.s2 = ((rxbuf[5] >> 4) & 0x0003);

    rcCtrl.wheel = (rxbuf[16] | rxbuf[17] << 8) - 1024;
}

static THD_WORKING_AREA(uart_dbus_thread_wa, 512);
static THD_FUNCTION(uart_dbus_thread, p)
{
    (void)p;
    chRegSetThreadName("uart dbus receiver");

    uartStart(UART_DRIVER, &uartcfg);
    msg_t rxmsg;
    systime_t timeout = TIME_MS2I(4U);
    uint32_t count = 0;

    while (!chThdShouldTerminateX())
    {
        uartStopReceive(UART_DRIVER);
        uartStartReceive(UART_DRIVER, DBUS_BUFFER_SIZE, rxbuf);
        chSysLock();
        rxmsg = chThdSuspendTimeoutS(&uart_dbus_thread_handler, timeout);
        chSysUnlock();

        if (rxmsg == MSG_OK)
        {
            chSysLock();
            processRxData();
            chSysUnlock();
        }
        else
        {
            timeout = TIME_MS2I(4U);
        }
        count++;
    }
}

RC_control_t *RCGet(void) { return &rcCtrl; }

void RCInit(void)
{
    RCReset();
    chThdCreateStatic(uart_dbus_thread_wa,
                      sizeof(uart_dbus_thread_wa),
                      NORMALPRIO + 7,
                      uart_dbus_thread,
                      NULL);
}