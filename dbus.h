#ifndef  _DBUS_H_
#define _DBUS_H_
#include "stdint.h"
    #define RC_CH_VALUE_MIN             ((uint16_t)364)
    #define RC_CH_VALUE_OFFSET          ((uint16_t)1024)
    #define RC_CH_VALUE_MAX             ((uint16_t)1684)
    #define DBUS_BUFFER_SIZE            ((uint8_t)18)
    #define UART_DRIVER                 &UARTD2
    void RCInit(void);
typedef struct{
    int16_t channel0;
    int16_t channel1;
    int16_t channel2;
    int16_t channel3;
    uint8_t s1;
    uint8_t s2;

    int16_t wheel;   
}RC_control_t;

typedef enum{
    RC_UNCONNECTED = 0,
    RC_LOCKED,
    RC_UNLOCKING,
    RC_UNLOCKED
}rc_state_t;
rc_state_t rcStateGet(void);
RC_control_t* RCGet(void);
void RCinit(void);
#endif