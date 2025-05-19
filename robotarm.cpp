#include "mbed.h"
#define _baud 1000000
const double x=200;
const double y=200;
const double z=200;
const double l2=161.75;
const double l3=230;
double theta1 =0;
double theta2=0;
double theta3=0;
Serial pc(USBTX, USBRX);
Serial AX12(PA_2, PA_3);
DigitalOut CHANGE(PC_5);
/*
double px=cos(theta1)*(l2*cos(theta2)+l3*cos(theta2+theta3));
double py=sin(theta1)*(l2*cos(theta2)+l3*cos(theta2+theta3));
double pz=l2*sin(theta2)+l3*sin(theta2+theta3);*/
int main(){
    int degree(int degree, int ID);
    AX12.baud(_baud);
    const int id = 15;
    CHANGE = 1;
    theta3 = acos((x*x+y*y+z*z-l2*l2-l3*l3)/(2*l2*l3));
    theta1 = atan2(y,x)
    if(x*x+y*y==0)theta1=0
    theta2 = atan2()
    while(1){
        degree(0, id);
        wait(1);
        degree(90, id);
        wait(1);
        degree(180, id);
        wait(1);
        degree(270, id);
        wait(1);
    }
}

int degree(int degree, int ID){
    uint8_t buff[16];
    uint8_t status[8];
    uint16_t sum = 0;
    uint16_t goal = (1023 * degree) / 300;
    buff[0] = 0xff;
    buff[1] = 0xff;
    buff[2] = ID;
    buff[3] = 5;
    buff[4] = 0x03;
    buff[5] = 0x1e;
    buff[6] = goal & 0xff;
    buff[7] = goal >> 8;
    sum += buff[2] + buff[3] + buff[4] + buff[5] + buff[6] + buff[7];
    buff[8] = 0xff & sum;
    //-----------データの送信
    CHANGE = 0;
    for(int i = 0 ; i < 9 ; i++) AX12.putc(buff[i]);
    //-----------データ受信
    wait_us(20);
    CHANGE = 1;
    int timeout = 0;
    int len = 0;
    while ((timeout < 100) && (len < 6)) {
        if (AX12.readable()) {//正しくデータが読み取れているか
            status[len] = AX12.getc();
            len++;
            timeout = 0;
        }
        wait(1.0/_baud);
        timeout++;
    }
    //-------------
    return status[4];
}
