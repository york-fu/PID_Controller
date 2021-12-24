#!/usr/bin/env python
# -*- coding: utf-8 -*-

class PositionPID:
    def __init__(self, p=0.0, i=0.0, d=0.0):
        self.__kp, self.__ki, self.__kd = p, i, d
        self.__error, self.__last_error = 0.0, 0.0
        self.__err_integral = 0.0
        self.__integral_min, self.__integral_max = -1e8, 1e8

    def __integral_limiter(self, integral):
        integral = self.__integral_max if integral > self.__integral_max else integral
        integral = self.__integral_min if integral < self.__integral_min else integral
        return integral

    def set_parameter(self, p, i, d):
        self.__kp, self.__ki, self.__kd = p, i, d

    def set_integral_limit(self, iMin, iMax):
        self.__integral_min, self.__integral_max = iMin, iMax

    def clear_integral(self):
        self.__err_integral = 0

    def run(self, error):
        self.__error = error
        self.__err_integral = self.__err_integral + self.__error
        self.__err_integral = self.__integral_limiter(self.__err_integral)
        out = self.__kp*self.__error + self.__ki*self.__err_integral + self.__kd*(self.__error-self.__last_error)
        self.__last_error = self.__error
        return out


if __name__ == "__main__":
    class Particle:
        def __init__(self, m=1.0) -> None:
            self.a = 0.0
            self.v = 0.0
            self.p = 0.0
            self.__m = m # kg

        def update(self, f, dt):
            self.a = f / self.__m
            self.v = self.v + (self.a * dt)
            self.p = self.p + (self.v * dt)

    pid = PositionPID(50, 0.1, 0.0)
    pid.set_integral_limit(-1e8, 1e8)

    dt = 0.01
    target = 1
    measure = 0
    output = 0

    sys = Particle(1.0)
    for i in range(50):
        measure = sys.v
        err = target - measure
        output = pid.run(err)
        sys.update(output, dt)
        print("step(%d): %f, %f, %f"%(i+1, measure, err, output))