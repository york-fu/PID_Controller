#include "pid_controller.h"

#define LIMIT(v, min, max) ((v) > (max) ? (max) : ((v) < (min) ? (min) : (v)))

void PID_setZero(PIDParam_t *pid)
{
  pid->kp = 0;
  pid->ki = 0;
  pid->kd = 0;
  pid->ek = 0;
  pid->ek_1 = 0;
  pid->ek_2 = 0;
  pid->uk = 0;
  pid->integral = 0;
  pid->integral_limit[0] = 0;
  pid->integral_limit[1] = 0;
  pid->uk_limit[0] = 0;
  pid->uk_limit[1] = 0;
}

void PID_setGain(PIDDataType_t kp, PIDDataType_t ki, PIDDataType_t kd, PIDParam_t *pid)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void PID_setIntegralLimit(PIDDataType_t min_integral, PIDDataType_t max_integral, PIDParam_t *pid)
{
  if(max_integral < min_integral)
  {
    printf("Wrong parameter!\n");
    printf("On line %d of %s file.\n", __LINE__, __FILE__);
    return;
  }
  pid->integral_limit[0] = min_integral;
  pid->integral_limit[1] = max_integral;
}

void PID_setOutputLimit(PIDDataType_t min_uk, PIDDataType_t max_uk, PIDParam_t *pid)
{
  if(max_uk < min_uk)
  {
    printf("Wrong parameter!\n");
    printf("On line %d of %s file.\n", __LINE__, __FILE__);
    return;
  }
  pid->uk_limit[0] = min_uk;
  pid->uk_limit[1] = max_uk;
}

PIDDataType_t PID_positionController(PIDParam_t *pid, PIDDataType_t err)
{
  pid->ek = err;
  pid->integral += pid->ek;
  pid->integral = LIMIT(pid->integral, pid->integral_limit[0], pid->integral_limit[1]);
  pid->uk = pid->kp * pid->ek + pid->ki * pid->integral + pid->kd * (pid->ek - pid->ek_1);
  pid->ek_1 = pid->ek;
  return pid->uk;
}

PIDDataType_t PID_incrementController(PIDParam_t *pid, PIDDataType_t err)
{
  pid->ek = err;
  pid->uk += pid->kp * (pid->ek - pid->ek_1) + pid->ki * pid->ek + pid->kd * (pid->ek - 2 * pid->ek_1 + pid->ek_2);
  pid->uk = LIMIT(pid->uk, pid->uk_limit[0], pid->uk_limit[1]);
  pid->ek_2 = pid->ek_1;
  pid->ek_1 = pid->ek;
  return pid->uk;
}
