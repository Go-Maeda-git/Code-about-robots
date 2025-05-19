#include <iostream>
using namespace std;
#include "mbed.h"
#include "MOTOR/motor.hpp"
#include "PID/PID.hpp"
#include "ROTARYINC/rotaryinc.hpp"
#include "BNO055.hpp"
#include "sbdbt.hpp"
//-----------------
#define kp0 0.00002//0.0006
#define ki0 0.000002//0.00001
#define kd0 0.0000000//0.00001

#define kp 0.000003//0.0004
#define ki 0.0000002
#define kd 0.00000000

#define dt 0.0001//0.01
#define resolution 512.0//分解能
#define stick_n 64.0//スティックに触れていないときの値
#define pai 3.14159265358979
#define diametar 1.07//直径
#define d_widht  0.05//タイヤ間の距離
//------------------
Timer time_;
PID pid[2] = {
        PID(kp0,ki0,kd0,dt),
        PID(kp,ki,kd,dt)
        };
sbdbt sbd(PA_0,PA_1);
BNO055 bno(PB_3,PB_10);
Motor mot[2] = {
    Motor(PB_7,PB_6),
    Motor(PC_9,PC_8)
};
rotaryinc rot[2] = {
    rotaryinc(PC_0,PC_1,diametar,resolution,dt),
    rotaryinc(PC_5,PA_12,diametar,resolution,dt)
};


int main()
{

bool A = false;
bool B = false;

int rotate = 0;
    bno.reset();
    bno.setmode(OPERATION_MODE_IMUPLUS);

    double vx = 0.0;
    double vy = 0.0;
    double vx_tar_limit = 10;
    double vy_tar_limit = 10;
    double rvx = 0.0;
    double tar[2] = {0.0};//単位をつける
    
    double num = 0.0;
    double speed[2] = {0.0};
    double theta = 0.0;
    double t = 0.0;
    bool flag = 1;
    int count = 0;
    //--------自己位置推定関連
    int mode_up = 0;
    bool mode_down =0;
    double pl_now0 = 0.0;
    double pl_now1 = 0.0;
    double pl_tar = 0.0;
    bool turn_r = 0;
    bool turn_l = 0;
    double now_raw_r = 0.0;
    double tar_raw_r = 0.0;
    double now_raw_l = 0.0;
    double tar_raw_l = 0.0;
    int right_tar = 0;
    double a = 0.0;
    double a_theta = 0.0;
    double b_theta = 0.0;
    int mode = 0;
    double then_raw =0.0;
    //-------------------------
    //int stick_mode =0;//後で追加する予定
    time_.start();
    while(1)
    {
        if(sbd.square())
        {
            flag = 0;  
        }
        bno.get_angles();
        if(flag == 1)
        {
/*gyro*/

double raw_yaw = 360 - bno.euler.yaw;

if(!A and 0 <= raw_yaw and raw_yaw <= 90) A = true;
if(!B and 270 <= raw_yaw and raw_yaw <= 360) B = true;
if(90 < raw_yaw and raw_yaw < 270) {
    A = false;
    B = false;
}
if(A and 270 <= raw_yaw and raw_yaw <= 360){
    rotate--;
    A = false;
    B = true;
}
else if(B and 0 <= raw_yaw and raw_yaw <= 90){
    rotate++;
    A = true;
    B = false;
}

double theta = raw_yaw + 360*rotate;

        tar[0] = -(360-theta)*3;
        tar[1] = (360-theta)*3;
        printf("%f",theta);
        for(int i = 0; i < 2; i++)
        {
            speed[i] = rot[i].getspeed();
            pid[i].set(speed[i],tar[i]);
            pid[i].safety(0.3,-0.3);
            mot[i].revolution(pid[i].pwm);
        }
        while(time_.read() - t < dt);
        t = time_.read();
        }
        else
        {
        mot[0].revolution(0); 
        mot[1].revolution(0); 
        }
    }
    return 0;
}
