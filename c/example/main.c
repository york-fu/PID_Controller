#include <stdio.h>
#include <unistd.h>
#include "pid_controller.h"

typedef struct
{
  double a;
  double v;
  double p;
  double m; // kg
} SysState_t;

void system_setZero(SysState_t *state)
{
  state->a = 0;
  state->v = 0;
  state->p = 0;
}

void system_Update(double f, double dt, SysState_t *state)
{
  state->a = f / state->m;
  state->v += state->a * dt;
  state->p += state->v * dt;
}

int main(int argv, char **argc)
{
  double dt = 0.01;
  double measure = 0;
  double target = 1; // m/s
  double output = 0;
  double err = 0;
  SysState_t state;
  state.m = 1; // kg

  PIDParam_t pid;
  PID_setZero(&pid);
  PID_setGain(50, 0.1, 0.0, &pid);
  PID_setIntegralLimit(-1e8, 1e8, &pid);

  system_setZero(&state);
  for (size_t i = 0; i < 50; i++)
  {
    measure = state.v;
    err = target - measure;
    output = PID_positionController(&pid, err);
    system_Update(output, dt, &state);
    printf("step(%ld): %f, %f, %f\n", i+1, measure, err, output);
  }
  printf("\n");

  PID_setZero(&pid);
  PID_setGain(20, 1, 0.0, &pid);
  PID_setOutputLimit(-1e3, 1e3, &pid);

  system_setZero(&state);
  for (size_t i = 0; i < 50; i++)
  {
    measure = state.v;
    err = target - measure;
    output = PID_incrementController(&pid, err);
    system_Update(output, dt, &state);
    printf("step(%ld): %f, %f, %f\n", i+1, measure, err, output);
  }
  printf("\n");

  return 0;
}
