#ifndef MOTOR_
#define MOTOR_

#include <mbed.h>

#if 0
  ~example~
    Motor name(ピン名1,ピン名2)でこのクラスを使うことができるようになる
    name.revolution(pwmの値)でモータを回すことができる
    name.reset()でモータの値を0にする
#endif

class Motor
{
public:
    Motor(PinName pin_a,PinName pin_b);
    void revolution(double);
    void reset();
private:
    PwmOut motor1;
    PwmOut motor2;
};

#endif