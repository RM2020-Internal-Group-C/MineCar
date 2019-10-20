#ifndef __PID_H__

#define __PID_H__

enum
{
    LAST = 0,
    NOW = 1,
};

typedef struct
{
    float kp;
    float ki;
    float kd;
    float set;
    float get;
    float err[2];
    float integral;
    float out;
    float maxOut;
} pid_t;

void PIDInit(pid_t *pid, int maxOut, float kp, float ki, float kd);

// void PIDReset(pid_t pid, float kp, float ki, float kd);

float PIDSet(pid_t *pid, float get, float set);

extern pid_t pidWheel[4];
#endif