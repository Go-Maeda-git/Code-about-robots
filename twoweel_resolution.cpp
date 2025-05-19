#include <iostream>
using namespace std;
#include "BNO055.hpp"
#include "MOTOR/motor.hpp"
#include "PID/PID.hpp"
#include "ROTARYINC/rotaryinc.hpp"
#include "mbed.h"
#include "sbdbt.hpp"
//===================
#define kp0 0.00002  // 0.0006
#define ki0 0.000001 // 0.00001
#define kd0 0.000000 // 0.00001

#define kp 0.00002 // 0.0004
#define ki 0.000001
#define kd 0.000000

#define dt 0.0001        // 0.01
#define resolution 512.0 //分解能
#define stick_n 64.0     //スティックに触れていないときの値
#define pai 3.14159265358979
#define diametar 1.07 //直径
#define d_widht 0.05  //タイヤ間の距離
//------------------
PID pid[2] = {PID(kp0, ki0, kd0, dt), PID(kp, ki, kd, dt)};
sbdbt sbd(PA_0, PA_1);
BNO055 bno(PB_3, PB_10);
Motor mot[2] = {Motor(PB_7, PB_6), Motor(PC_9, PC_8)};
rotaryinc rot[2] = {rotaryinc(PC_0, PC_1, diametar, resolution, dt),
                    rotaryinc(PC_5, PA_12, diametar, resolution, dt)};
//----------------global変数
double tar[2] = {0.0};
int num = 0;
double pl_now0 = 0.0;
int order = 100;
int mode_up = 0;
double raw_sum = 0;
double mode_turn = 0;
  double target = 0;
//-----------------関数
//up(パルスの目標値,現在の角度,現在のパルス)
int up(double plst, double theta_, double pl) {

  if (mode_up == 0) {
    pl_now0 = pl;
    mode_up = 1;
  }
  if (mode_up == 1) {
    tar[1] = -40 + (raw_sum - theta_) * 5;
    tar[0] = -40 - (raw_sum - theta_) * 5;
  }
  if (mode_up == 1) {
    if (pl_now0 + plst > pl) {

      num = 1;
      order++;
      mode_up = 0;
    }
  }
  return 0;
}
//-----------------------------------
int buck(double plst, double theta_, double pl) {

  if (mode_up == 0) {
    pl_now0 = pl;
    mode_up = 1;
  }
  if (mode_up == 1) {
    tar[1] = +40 + (raw_sum - theta_) * 5;
    tar[0] = +40 - (raw_sum - theta_) * 5;
  }
  if (mode_up == 1) {
    if (pl_now0 + plst < pl) {

      num = 1;
      order++;
      mode_up = 0;
    }
  }
  return 0;
}
//------------------------------------
int turn(double theta_t, double theta_1) {
  if (mode_turn == 0) {
    target = theta_1 + theta_t;
    mode_turn = 1;
    raw_sum = target;
  }
  if (mode_turn == 1) {
    if (theta_t < 0) {
      tar[0] = 40;
      tar[1] = -40;
    } else if (theta_t > 0) {
      tar[0] = -40;
      tar[1] = 40;
    }
  }
  if (mode_turn == 1)
  {
    if (theta_t < 0) 
    {
      if (theta_1 <= target) 
      {
        order++;
        mode_turn = 0;
        num = 1;
      }
    }
    else{
      if (theta_1 >= target) 
      {
        order++;
        mode_turn = 0;
        num = 1;
      }
    }
  }
  return 0;
}
//----------------------
Timer time_;

int main() {

  bool A = false;
  bool B = false;

  int rotate = 0;
  bno.reset();
  bno.setmode(OPERATION_MODE_IMUPLUS);
  double speed[2] = {0.0};
  double theta = 0.0;
  double t = 0.0;
  bool flag = 1;
  int count = 0;
  //--------自己位置推定関連
  double a = 0.0;
  int mode = 0;
  double then_raw = 0.0;
  int abc = 0;
  //-------------------------
  time_.start();
  raw_sum = theta;
  while (1) {
    if (sbd.triangle())
      order = 0;
    if (sbd.square()) {
      flag = 0;
    }
    bno.get_angles();

    if (flag == 1) {
      for (int i = 0; i < 2; i++)
      
       {
        speed[i] = rot[i].getspeed();
        pid[i].set(speed[i], tar[i]);
        pid[i].safety(0.3, -0.3);
        mot[i].revolution(pid[i].pwm);
        count = 0;
      }
      /*gyro*/
      double raw_yaw = 360 - bno.euler.yaw;

      if (!A and 0 <= raw_yaw and raw_yaw <= 90)
        A = true;
      if (!B and 270 <= raw_yaw and raw_yaw <= 360)
        B = true;
      if (90 < raw_yaw and raw_yaw < 270) {
        A = false;
        B = false;
      }
      if (A and 270 <= raw_yaw and raw_yaw <= 360) {
        rotate--;
        A = false;
        B = true;
      } else if (B and 0 <= raw_yaw and raw_yaw <= 90) {
        rotate++;
        A = true;
        B = false;
      }

      double theta = raw_yaw + 360 * rotate;
      /*end*/
      //---------初期角度代入
      if (abc == 0) {
        raw_sum = theta;
        abc = 1;
      }
      //--------停止
      if (num == 1) {
        tar[0] = 0;
        tar[1] = 0;
        pid[0].pwm = 0;
        pid[1].pwm = 0;
        mot[0].revolution(0);
        mot[1].revolution(0);
        mode_up = 0;
        mode_turn = 0;
        num = 0;
      }
      //-------------------
      switch (order) {
      case 0:
        up(-512, theta, rot[0].getpulse());
        break;
      case 1:
        buck(512, theta, rot[0].getpulse());
        break;
      case 2:
        turn(180, theta);
        break;
      }
      //---------------------
      while (time_.read() - t < dt)
        ;
      t = time_.read();
    } else {
      mot[0].revolution(0);
      mot[1].revolution(0);
    }
  }
  return 0;
}
