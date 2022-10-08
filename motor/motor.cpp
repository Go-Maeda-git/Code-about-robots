#include "motor.hpp"

Motor::Motor(PinName pin_a,PinName pin_b) : motor1(pin_a),motor2(pin_b)
{
    motor1.period_us(2048);
    motor2.period_us(2048);
}

void Motor::revolution(double pwm)
{
    if(pwm == 0.0)
    {
        motor1 = 0.0;
        motor2 = 0.0;
    }
    else if(pwm > 0.0)
    {
        motor1 = pwm;
        motor2 = 0.0;
    }
    else if(pwm < 0.0)
    {
        motor1 = 0.0;
        motor2 = -pwm;
    }
}

void Motor::reset()
{
    motor1 = 0.0;
    motor2 = 0.0;
}