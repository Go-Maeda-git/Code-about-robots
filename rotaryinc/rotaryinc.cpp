#include "rotaryinc.hpp"

rotaryinc::rotaryinc(PinName pinA,PinName pinB,double diameter,double resolution,double dt)
{
    Diameter = diameter;
    Resolution = resolution;
    dt_ = dt;
    init(pinA,pinB);
    pulse[0] = 0;
    pulse[1] = 0;
    speed = 0;
    timer.start();
}

void rotaryinc::init(PinName pin_a,PinName pin_b)
{
    pin_a_ = new InterruptIn(pin_a,PullUp);
    pin_b_ = new InterruptIn(pin_b,PullUp);
    pin_a_->rise(callback(this,&rotaryinc::riseA));
    pin_a_->fall(callback(this,&rotaryinc::fallA));
    pin_b_->rise(callback(this,&rotaryinc::riseB));
    pin_b_->fall(callback(this,&rotaryinc::fallB));
}

void rotaryinc::riseA()
{
    pin_b_ -> read() ? pulse[1]-- : pulse[1]++;
}

void rotaryinc::fallA()
{
    pin_b_ -> read() ? pulse[1]++ : pulse[1]--;
}

void rotaryinc::riseB()
{
    pin_a_ -> read() ? pulse[1]++ : pulse[1]--;
}

void rotaryinc::fallB()
{
    pin_a_ -> read() ? pulse[1]-- : pulse[1]++;
}

double rotaryinc::getpulse()
{
    return pulse[1]/4;
}

double rotaryinc::getspeed()
{
    speed = (M_PI * Diameter) * (pulse[1]/4 - pulse[0]) / Resolution / dt_;
    pulse[0] = pulse[1]/4;
    return speed;
}

double rotaryinc::gettheta()
{
    theta[1] = pulse[1]/4 / Resolution * 360;
    return theta[1];
}

double rotaryinc::getomega()
{
    omega = (theta[1] - theta[0]) / dt_;
    theta[0] = theta[1];
    return omega;
}

double rotaryinc::getrevolution()
{
    return pulse[1]/4 / Resolution;
}

void rotaryinc::loop()
{
    time = timer.read();
    while(timer.read() - time <= dt_);
}

rotaryinc::~rotaryinc(){
    pin_a_->disable_irq();
    pin_b_->disable_irq();
    delete pin_a_;
    delete pin_b_;
}