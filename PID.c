#include "PID.h"

pid_t pidWheel[4];

static void clamp(float *a, float max)
{
    if (*a > max)
    {
        *a = max;
    }
    if (*a < -max)
    {
        *a = -max;
    }
}

void PIDInit(pid_t *pid, int maxOut, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->maxOut = maxOut;
}

// float absp(float i) { return (i < 0) ? -i : i; }

float PIDSet(pid_t *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->errNOW = set - get;
    pid->p = pid->errNOW * pid->kp;
    pid->i += pid->errNOW * pid->ki;
    // clamp(&pid->i, 500);
    pid->d = (pid->errNOW - pid->errLAST) * pid->kd;
    pid->out = pid->p + pid->i + pid->d;
    pid->errLAST = pid->errNOW;
    clamp(&pid->out, pid->maxOut);
    return pid->out;
}