#ifndef PID_H
#define PID_H

typedef struct {
    /* Controller Gains */
    float Kp;
    float Ki;
    float Kd;

    /* sampling time and corner frequency */
    float Ts; 
    float N;
    
    /* Output Limits */
    float limMin;
    float limMax;

    /* Controller "memory" */
    float integrator;
    float prevError;        // for integrator
    float differentiator;
    float prevMeasurement;  // for differentiator

    /* differentiator coefficients */
    float alpha;
    float beta;

    /* Controller Ouput */
    float out;
} PIDController;

void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif