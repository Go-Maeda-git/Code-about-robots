#include <iostream>
using namespace std;
#include "mbed.h"
#include "motor/motor.hpp"
#include "SBDBT/SBDBT.hpp"
//#include "arrc_mbed/scrp_slave.hpp"

//-----定数

const double stick_neutral=64.0;//スティックを動かしてない時の値
const double tarx=0.48;//xの目標値
const double tary = 0.12;//yの目標値
               
Motor mot[2] = {
     Motor(PC_8,PC_9),
    Motor(PB_8,PB_9)
};

DigitalOut led[2] = {
    DigitalOut (PA_13),
    DigitalOut (PB_14),
};

InterruptIn slit(PA_11);//Interrutlnは割り込み処理をするための宣言です。
Serial pc1(USBTX,USBRX);

int main()
{
    pc1.baud(9600);
    sbdbt sbd(PA_0,PA_1);                      
    double speed[2] = {0.0};
    double vx = 0.0;
    double vy = 0.0;
    double vx_tar_limit = 1000.0;
    double vy_tar_limit = 1000.0;
    double pwm[2] = {0.0};

    while(1) {
        vx = sbd.rsx();
        vy = sbd.rsy();
        pwm[0] = tarx/stick_neutral * (127.0 - vx - stick_neutral); //rsxはスティックのX方向の値のこと
        pwm[1] = tary/stick_neutral * (127.0 - vy - stick_neutral); //rsyはスティックのY方向の値のこと
       /*if(slit.rise() || slit.fall())
        {
            if(pwm<0)
            {
                pwm = 0;
            }
        }*/        
        for(int i = 0; i < 2; i++) 
          {
            mot[i].revolution(pwm[i]);  //pwmの値を引数として送ってモータが回る
            pc1.printf("pwm[%d]=%f",i,pwm[i]);
          }
          pc1.printf("\n");
       
    }
}
