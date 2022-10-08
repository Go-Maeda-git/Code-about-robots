#include "PID.hpp"
PID::PID(double kp,double ki,double kd,double dt) : Kp(kp),Ki(ki),Kd(kd),dt(dt)
{
    pwm = 0;
    e[0] = 0;
    e[1] = 0;
    integral = 0;
}

double PID::set(double nowvalue,double tar)
{
    e[1] = tar - nowvalue;
    integral += (e[0] + e[1]) * dt / 2;
    getP = e[1];
    getI = integral;
    getD = (e[1] - e[0]) / dt;
    pwm += (getP * Kp) + (getI * Ki) + (getD * Kd);
    e[0] = e[1];
    return pwm;
}

void PID::safety(double max,double min)
{
    pwm >= max ? pwm = max : pwm = pwm;
    pwm <= min ? pwm = min : pwm = pwm;
}

void PID::reset()
{
    pwm = 0;
}

PID::~PID(){};