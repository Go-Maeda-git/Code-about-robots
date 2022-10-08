#include <iostream>
using namespace std;
#include "mbed.h"
#include "BNO055.hpp"
#include "motor.hpp"
#include "PID.hpp"
#include "rotaryinc.hpp"
#include "SBDBT.hpp"

#define kp 0.00005
#define ki 0.00000001
#define kd 0.00000001
#define dt 0.05
#define diameter 101.6
#define resolution 256.0

int main()
{
    BNO055 bno(D3,D6);
    sbdbt sbd(A0,A1);                       //アナログが対応しているピンを設定するべき
    bno.reset();                            //BNO初期化
    bno.setmode(OPERATION_MODE_IMUPLUS);    //角度を読み取れるようにするモード？

    Motor mot[4] = {
        Motor(PC_8,PC_9),
        Motor(PB_6,PB_7),
        Motor(PB_4,PB_5),
        Motor(PB_13,PB_14)
    };

    PID pid[4] = {
        PID(kp,ki,kd,dt),
        PID(kp,ki,kd,dt),
        PID(kp,ki,kd,dt),
        PID(kp,ki,kd,dt)
    };

    rotaryinc rot[4] = {
        rotaryinc(PC_5,PA_12,diameter,resolution,dt),
        rotaryinc(PC_1,PC_0,diameter,resolution,dt),
        rotaryinc(PA_9,PA_8,diameter,resolution,dt),
        rotaryinc(PA_7,PA_6,diameter,resolution,dt)
    };

    double Theta = 0.0;
    double tar[4] = {0.0};
    double speed[4] = {0.0};
    double vx = 0.0;
    double vy = 0.0;
    double rotation = 0.0;
    double vx_tar_limit = 1000.0;
    double vy_tar_limit = 1000.0;
    double rotation_tar_limit = 1000.0;
    bool mode = 1;

    while(1) {

        bno.get_angles();                       //角度を取得
        Theta = bno.euler.yaw * (M_PI / 180);   //弧度法に直す

        for(int i = 0; i < 4; i++) {
            speed[i] = rot[i].getspeed(); //モータのmm/sをrotaryincのクラスから取得
        }

        vx = (vx_tar_limit / 64) * (sbd.lsx() - 64);                        //X方向の最大目標値決め
        vy = (vy_tar_limit / 64) * (sbd.lsy() - 64);                        //Y方向の最大目標値決め
        rotation = (rotation_tar_limit / 255) * (sbd.r2An() - sbd.l2An());  //回転運動の最大目標値決め



        //目標値決め
        if(mode){
            tar[0] =  vx * cos(M_PI/4 + Theta) + vy * sin(M_PI/4 + Theta) + rotation;
            tar[1] = -vx * sin(M_PI/4 + Theta) + vy * cos(M_PI/4 + Theta) + rotation;
            tar[2] = -vx * cos(M_PI/4 + Theta) - vy * sin(M_PI/4 + Theta) + rotation;
            tar[3] =  vx * sin(M_PI/4 + Theta) - vy * cos(M_PI/4 + Theta) + rotation;
        }

        else {
            tar[0] =  vx * cos(M_PI/4) + vy * sin(M_PI/4) + rotation;
            tar[1] = -vx * sin(M_PI/4) + vy * cos(M_PI/4) + rotation;
            tar[2] = -vx * cos(M_PI/4) - vy * sin(M_PI/4) + rotation;
            tar[3] =  vx * sin(M_PI/4) - vy * cos(M_PI/4) + rotation;
        }

        for(int i = 0; i < 4; i++) {
            pid[i].set(speed[i],tar[i]);    //現在値と目標値を引数として送ってクラス内でPID制御がされる
            pid[i].safety(0.5,-0.5);        //pwmの値が大きくなりすぎないようにpwmの最大値を設定しておく
            mot[i].revolution(pid[i].pwm);  //PID制御をしてでてきたpwmの値を引数として送ってモータが回る
        }

        cout << "tar = " << tar[0] << "  speed = " << speed[0] << endl;
        rot[0].loop(); //dt秒ごとに動くプログラムになる

    }
}