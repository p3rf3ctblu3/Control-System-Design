#include <PID.h>

void PIDController_Init(PIDController *pid) {

    pid->Ts = 0.001f;
    pid->N = (2 * 3.14f) / (10 * pid->Ts); 

    pid->alpha = 1.0f / (1.0f + pid->N * pid->Ts);
    pid->beta = pid->Kd * pid->N * pid->alpha; 

    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement){

    float error = setpoint - measurement;
    float proportional = pid->Kp * error; 
    
    pid->integrator += pid->Ki * pid->Ts * error;

    /* Integrator anti-windup */
    if (pid->integrator > pid->limMax) pid->integrator = pid->limMax;
    else if (pid->integrator < pid->limMin) pid->integrator = pid->limMin;
    
    pid->differentiator = (pid->differentiator * pid->alpha) + 
                       (pid->beta) * (pid->prevMeasurement - measurement);

    /* update previous error */
    pid->prevError = error; 

    float out = proportional + pid->integrator + pid->differentiator;

    /* final output clamping */

    if (out > pid->limMax) out = pid->limMax;
    else if (out < pid->limMin) out = pid->limMin;
    
    pid->prevMeasurement = measurement;
    pid->out = out; 
    return out; 
}

