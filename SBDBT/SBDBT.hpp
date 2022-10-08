#ifndef SBDBT_H
#define SBDBT_H
#include "mbed.h"

//data[1]
#define CIRCLE 0x01
#define L1     0x02
#define L2     0x04
#define R1     0x08
#define R2     0x10
#define START  0x20
#define SELECT 0x40

//data[2]
#define UP     0x01
#define DOWN   0x02
#define LEFT   0x04
#define RIGHT  0x08
#define TRI    0x10
#define CROSS  0x20
#define SQUARE 0x40

/*SBDBTを用いてPS3コントローラ(DS3)を使う
  sbdbt sb(A0, A1);                           マイコン ~ PC間のボーレート指定なし(9600)
  sbdbt sb(A0, A1, 115200);                   マイコン ~ PC間のボーレート指定あり
*/

class sbdbt {
public:
    sbdbt(PinName TX, PinName RX);           //マイコン ~ PC間のボーレート指定なし(9600)
    sbdbt(PinName TX, PinName RX, int baud); //マイコン ~ PC間のボーレート指定あり
    void get(void); 
    void button_state();                     //全ボタン, スティックの状態表示 
    void data_print();
    bool neutral();
    bool up();
    bool down();
    bool left();
    bool right();
    bool tri();
    bool cross();
    bool square();
    bool circle();
    bool l1();
    bool l2();
    bool r1();
    bool r2();
    bool start();
    bool select();
    char lsx();
    char lsy();
    char rsx();
    char rsy();
    char l2An();
    char r2An();
    bool command();
    ~sbdbt();
private:
    Serial *dev;
    char *data;
    char *PS3_Data;
    char i;
    char pos;
    char val;
    char len;
};
#endif