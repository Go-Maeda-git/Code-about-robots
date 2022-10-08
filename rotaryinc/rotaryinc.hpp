#ifndef ROTARY_INC_
#define ROTARY_INC_
#include <mbed.h>
#define M_PI 3.1415926535

#if 0 
  ~example~                                                      
    rotaryinc name(ピン名1,ピン名2,オムニの直径,パルスの分解能,一回のプログラムの時間(Δt))でこのクラスを使うことができる
    name.getpulse()でパルスの値を取得できる
    name.getspeed()でモータの速さを取得できる
　  name.loop(一回のプログラムの時間(Δt))でΔt秒ごとのプログラムにできる
#endif

class rotaryinc
{
public:
    rotaryinc(PinName pinA,PinName pinB,double,double,double);
    ~rotaryinc();
    Timer timer;
    double getpulse();
    double getspeed();
    double gettheta();
    double getomega();
    double getrevolution();
    void loop();
private:
    InterruptIn *pin_a_,*pin_b_;
    double time;
    double pulse[2];
    double speed;
    void init(PinName,PinName);
    void riseA(void);
    void riseB(void);
    void fallA(void);
    void fallB(void);
    double theta[2];
    double omega;
    double Resolution;
    double Diameter;
    double dt_;
    double revolution_num;
};

#endif