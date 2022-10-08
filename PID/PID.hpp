#ifndef PID_
#define PID_

#if 0
  ~example~
    PID name(kpの値,kiの値,kdの値,一回のプログラムの時間(Δt))でこのクラスを使うことができる
    name.set(現在値,目標値)でPID制御の計算をしてくれる
    name.safety(最大値,最小値)でpwmの値が設定した値を超えないようにする(大事)
    name.pwmで求めたpwmの値を使うことができる
    name.reset()でpwmの値を0にする
#endif

class PID
{
public:
  PID(double kp,double ki,double kd,double dt);
  ~PID();
  double set(double,double); 
  void safety(double,double);
  void reset();
  double pwm;
private:
   double integral;
   double e[2];
   double Kp;
   double Ki;
   double Kd;
   double dt;
   double getP;
   double getI;
   double getD;
};

#endif