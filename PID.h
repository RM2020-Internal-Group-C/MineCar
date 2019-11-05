#ifndef __PID_H__

#define __PID_H__

typedef struct
{
    float kp;
    float ki;
    float kd;
    float set;
    float get;
    float errNOW;
    float errLAST;
    float out;
    float maxOut;
    float p;
    float i;
    float d;
    int pDirection;
    int nDirection;
} pid_t;
//Initiallize pid: maxOut:maximum outpus, Kp, Ki, Kd 
void PIDInit(pid_t *pid, int maxOut, float kp, float ki, float kd);

float PIDSet(pid_t *pid, float set , int last, int now);


extern pid_t pidmotor[2];
#endif