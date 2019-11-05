#include "PID.h"

pid_t pidmotor[2] = {{0}, {0}};

static void limit(float *a, float max)
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
    pid->pDirection = 0;
    pid->nDirection = 0;
}

float clamp(float i) { return (i < 0) ? -i : i; }

float PIDSet(pid_t *pid, float set , int last, int now)
{
    if((now - last) < (8191 - now + last))
    {
        pid->pDirection += (now - last);
    }
    else if((now - last) > (8191 - now + last)){
        pid->nDirection += (8191 - now + last);
    }
    pid->get = pid->pDirection;
    pid->set = set;
    pid->errNOW = set - pid->pDirection;
    pid->p = pid->errNOW * pid->kp;
    pid->i += pid->errNOW * pid->ki;
    pid->d = (pid->errNOW - pid->errLAST) * pid->kd;
    pid->out = pid->p + pid->i + pid->d;
    pid->errLAST = pid->errNOW;
    limit(&pid->out, pid->maxOut);
    return pid->out;
}

