#include "SBDBT.hpp"
#include "mbed.h"

Serial pc(USBTX,USBRX);

sbdbt::sbdbt(PinName TX, PinName RX){
    i = 0;
    pos = 0;
    val = 0x80;
    len = 10;
    data = new char[len];
    PS3_Data = new char[len];
    dev = new Serial(TX, RX);
    dev -> baud(9600);
    dev -> attach(callback(this,&sbdbt::get),Serial::RxIrq);
}

sbdbt::sbdbt(PinName TX, PinName RX, int baud){
    i = 0;
    pos = 0;
    val = 0x80;
    len = 10;
    data = new char[len];
    PS3_Data = new char[len];
    dev = new Serial(TX, RX);
    dev -> baud(9600);
    pc.baud(baud);
    dev -> attach(callback(this,&sbdbt::get),Serial::RxIrq);
}

void sbdbt::get(){
    data[i] = dev -> getc();
    if(data[i] == val){
        if(i != pos){
            data[pos] = val;
            i = pos + 1;
        }
        else{
            i++;
        }
    }
    else{
        if(i == pos);
        else{
            i++;
        }
    }
    if(i >= len){
        PS3_Data[0] = data[0];
        PS3_Data[1] = data[1];
        PS3_Data[2] = data[2];
        PS3_Data[3] = data[3];
        PS3_Data[4] = data[4];
        PS3_Data[5] = data[5];
        PS3_Data[6] = data[6];
        PS3_Data[7] = data[7];
        PS3_Data[8] = data[8];
        PS3_Data[9] = data[9];
        i = 0;
    }
}

void sbdbt::button_state(){pc.printf("UP: %d,  DOW: %d,  RIG: %d,  LEF: %d,  TRI: %d,  CRO: %d,  CIR: %d,  SQU: %d,  L1: %d,  L2: %d,  R1: %d,  R2: %d,  STA: %d,  SEL: %d  LSX: %3d,  LSY: %3d,  RSX: %3d,  RY: %3d,  L2An: %3d,  R2An: %3d\n",up(), down(), right(), left(), tri(), cross(), circle(), square(), l1(), l2(), r1(), r2(), start(), select(), lsx(), lsy(), rsx(), rsy(), l2An(), r2An());}

void sbdbt::data_print(){pc.printf("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n",PS3_Data[0], PS3_Data[1], PS3_Data[2], PS3_Data[3], PS3_Data[4], PS3_Data[5], PS3_Data[6], PS3_Data[7], PS3_Data[8], PS3_Data[9]);}

bool sbdbt::neutral(){
    if(data[9] == 0) return 1;
    return 0;
}

bool sbdbt::up(){
    if(data[2] & UP) return 1;
    return 0;
}

bool sbdbt::down(){
    if(data[2] & DOWN) return 1;
    return 0;
}

bool sbdbt::left(){
    if(data[2] & LEFT) return 1;
    return 0;
}

bool sbdbt::right(){
    if(data[2] & RIGHT) return 1;
    return 0;
}

bool sbdbt::tri(){
    if(data[2] & TRI) return 1;
    return 0;
}

bool sbdbt::cross(){
    if(data[2] & CROSS) return 1;
    return 0;
}

bool sbdbt::square(){
    if(data[2] & SQUARE) return 1;
    return 0;
}

bool sbdbt::circle(){
    if(data[1] & CIRCLE) return 1;
    return 0;
}
bool sbdbt::l1(){
    if(data[1] & L1) return 1;
    return 0;
}

bool sbdbt::l2(){
    if(data[1] & L2) return 1;
    return 0;
}

bool sbdbt::r1(){
    if(data[1] & R1) return 1;
    return 0;
}

bool sbdbt::r2(){
    if(data[1] & R2) return 1;
    return 0;
}

bool sbdbt::start(){
    if(data[1] & START) return 1;
    return 0;
}

bool sbdbt::select(){
    if(data[1] & SELECT) return 1;
    return 0;
}

char sbdbt::lsx(){return data[3];}

char sbdbt::lsy(){return data[4];}

char sbdbt::rsx(){return data[5];}

char sbdbt::rsy(){return data[6];}

char sbdbt::l2An(){return data[7];}

char sbdbt::r2An(){return data[8];}

sbdbt::~sbdbt(){
    delete dev;
    delete data;
    delete PS3_Data;
}