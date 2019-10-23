#include "PID.h"

pid_t pidWheel[4] ={{0},{0},{0},{0}};

static void limit(float *a, float max){
    if(*a>max){
        *a = max;
    }
    if(*a<-max){
        *a = -max;
    }
}

void PIDInit(pid_t *pid, int maxOut, float kp, float ki, float kd){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->maxOut = maxOut;
}

float PIDSet(pid_t *pid, float get, float set){
    pid->get = get;
    pid->set = set;
    pid->err[NOW] = set - get;
    pid->integral += pid->err[NOW];
    pid->out = (pid->err[NOW] * pid->kp + pid->integral * pid->ki + (pid->err[NOW] - pid->err[LAST]) * pid->kd); 
    limit(&pid->out, pid->maxOut);
    return pid->out;
}