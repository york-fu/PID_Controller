#ifndef _pid_controller_h_
#define _pid_controller_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <math.h>

#define PIDDataType_t double

typedef struct
{
  PIDDataType_t kp;
  PIDDataType_t ki;
  PIDDataType_t kd;
  PIDDataType_t ek;   // e(k)
  PIDDataType_t ek_1; // e(k-1)
  PIDDataType_t ek_2; // e(k-2)
  PIDDataType_t uk; //u(k)
  PIDDataType_t integral;
  PIDDataType_t integral_limit[2];
  PIDDataType_t uk_limit[2];
} PIDParam_t;

void PID_setZero(PIDParam_t *pid);
void PID_setGain(PIDDataType_t kp, PIDDataType_t ki, PIDDataType_t kd, PIDParam_t *pid);
void PID_setIntegralLimit(PIDDataType_t min_integral, PIDDataType_t max_integral, PIDParam_t *pid);
void PID_setOutputLimit(PIDDataType_t min_uk, PIDDataType_t max_uk, PIDParam_t *pid);
PIDDataType_t PID_positionController(PIDParam_t *pid, PIDDataType_t err);
PIDDataType_t PID_incrementController(PIDParam_t *pid, PIDDataType_t err);

#ifdef __cplusplus
}
#endif

#endif
