#include <GC5883.h>
#include <DueTimer.h>
#include <mymath.h>//自作ライブラリ
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
/*
どのラインセンサが反応してるか
0反応なし
1左
2右
3左と右
4後ろ
5左と後ろ
6右と後ろ
*/
//ピン番号
#define InA0 24//モタドラ
#define InB0 25//モタドラ
#define PWM0 3//モタドラ
#define InA1 23//モタドラ
#define InB1 22//モタドラ
#define PWM1 2//モタドラ
#define InA2 28//モタドラ
#define InB2 29//モタドラ
#define PWM2 5//モタドラ
#define SSIR 53//赤外線センサを読むマイコンのSSピン
#define SSUS 52//超音波センサを読むマイコンのSSピン
#define SSLN 43//ラインセンサを読むマイコンのSSピン
#define INTRPT 42//ライン割り込み(Interruptの略)
#define KICKER 40//ソレノイドを制御する電磁リレー
#define SWR 50//UI用のスイッチ
#define SWL 51//UI用のスイッチ
#define BT1 44//UI用のスイッチ
#define BT2 45//UI用のスイッチ
#define BT3 46//UI用のスイッチ
#define BALL 7//ボール検出センサ
//ピン番号以外のマクロ
#define KU 1//pidのパラメータ
#define TU 4.5//pidのパラメータ
#define NOM 4 //Number Of Modesの頭文字,モードの数
//------------------------------------------------
int posiRead() {
  //超音波センサを読むマイコンからロボットがどの区分にいるかを受け取る
  int recvd = 0;
  digitalWrite(SSUS, LOW);
  recvd = SPI.transfer(0);//()の中の数に意味はない
  digitalWrite(SSUS, HIGH);
  return recvd;
}
int lineRead() {
  //ラインセンサを読むマイコンからどのラインセンサが反応してるかを受け取る
  int recvd;
  digitalWrite(SSLN, LOW);
  recvd = SPI.transfer(0);//()の中の数に意味はない
  digitalWrite(SSLN, HIGH);
  return recvd;
}
int irRead() {
  //irセンサを読むマイコンから進行方向を受け取る
  int recvd;
  digitalWrite(SSIR, LOW);
  recvd = SPI.transfer(0);//()の中の数に意味はない

  digitalWrite(SSIR, HIGH);
  if (recvd == 255) {
    return 360;
  }
  recvd *= 10;
  recvd /= 7;
  return recvd;
}
mymath f;//sin,cosのテーブル参照の関数の自作クラス
void dir2out(int dir, int pwm, double *m0, double *m1, double *m2) {
  //進行方向からモーターの出力を決める
  double x, y;
  x = pwm * f.mycos(dir);
  y = pwm * f.mysin(dir);
  *m0 = x * f.mycos(300) + y * f.mysin(300);
  *m1 = -1 * x;
  *m2 = x * f.mycos(60) + y * f.mysin(60);
}
void move(double m0, double m1, double m2){
  //移動用モーターを動かす
  if (m0 > 0) {
    digitalWrite(InA0, HIGH);
    digitalWrite(InB0, LOW);
    analogWrite(PWM0, m0);
  }
  else {
    m0 *= -1;
    digitalWrite(InA0, LOW);
    digitalWrite(InB0, HIGH);
    analogWrite(PWM0, m0);
  }

  if (m1 > 0) {
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    analogWrite(PWM1, m1);
  }
  else {
    m1 *= -1;
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    analogWrite(PWM1, m1);
  }

  if (m2 > 0) {
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);
    analogWrite(PWM2, m2);
  }
  else {
    m2 *= -1;
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, HIGH);
    analogWrite(PWM2, m2);
  }
}
void stop() {
  //移動用モーターを止める
  digitalWrite(InA0, LOW);
  digitalWrite(InB0, LOW);
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, LOW);
  digitalWrite(InA2, LOW);
  digitalWrite(InB2, LOW);
}

int compassAddress = 0x42 >> 1;
int e = 0, e1 = 0, front;
double mani = 0, kp = KU;
GC5883 compass;
void timerHandler() {
//相手ゴール側に向くためのpd制御
  e1 = e;
  compass.init();
  e = front - (int)compass;
  if (e > 180) {
    e -= 360;
  }
  if (e < -180) {
    e += 360;
  }
  if ((e > -40) && (e < 40)) {
    kp = KU + 0.8;
  } else {
    kp = KU;
  }
  mani = 0.6 * kp * e + 0.125 * TU * (e - e1);
}

volatile int mode = 0;
void increase() {
  //ボタン入力の処理
  mode += 1;
  if (mode >= NOM) {
    mode = 0;
  }
}
void decrease() {
  //ボタン入力の処理
  mode -= 1;
  if (mode < 0) {
    mode = NOM;
  }
}
void startTimer() {
  //スイッチ入力でタイマー割り込みを再開させるための関数
  e=(int)compass;
  Timer3.attachInterrupt(timerHandler).setFrequency(60).start();
}
//------------------------------------------------
LiquidCrystal_I2C lcd(0x27,16,2);
void setup() {
//ioピンの設定
  pinMode(InA0, OUTPUT);
  pinMode(InB0, OUTPUT);
  pinMode(PWM0, OUTPUT);
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(SWR, INPUT);
  pinMode(SWL, INPUT);
  digitalWrite(InA0, HIGH);
  digitalWrite(InB0, HIGH);
  digitalWrite(InA1, HIGH);
  digitalWrite(InB1, HIGH);
  digitalWrite(InA2, HIGH);
  digitalWrite(InB2, HIGH);
//lcdの設定
  lcd.init();
  lcd.backlight();
//SPI通信の設定
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(16);
  SPI.setDataMode(SPI_MODE0);
  Serial.begin(9600);
  pinMode(SSIR, OUTPUT);
  pinMode(SSUS, OUTPUT);
  pinMode(SSLN, OUTPUT);
  digitalWrite(SSIR, HIGH);
  digitalWrite(SSUS, HIGH);
  digitalWrite(SSLN, HIGH);
//方位センサの設定
  compass.init();
  front = (int)compass;
//タイマー割り込みの設定
  Timer3.attachInterrupt(timerHandler).setFrequency(60).start();
//ピン割り込みの設定
  attachInterrupt(BT1, increase, RISING);
  attachInterrupt(BT3, decrease, RISING);
  attachInterrupt(SWR, startTimer, RISING);
}
int interval=0;
void loop() {
  //SPI通信のSSピンをHIGHにする
  digitalWrite(SSLN, HIGH);
  digitalWrite(SSIR, HIGH);
  digitalWrite(SSUS, HIGH);
  digitalWrite(KICKER,LOW);
  //変数の初期化
  double m0 = 0, m1 = 0, m2 = 0;
  int dir = irRead();
  int line = lineRead();
  int y = posiRead()%128;
  int x = y % 8;
  y /= 8;
  if (digitalRead(SWR) == LOW) {
    Timer3.stop();//I2C通信が中断されないようにタイマー割り込みを停止させる
    stop();//ロボットの動きを止める
    switch (mode) {
      case 0:
      //どのラインセンサが反応しているかを表示
        lcd.clear();
        lcd.setCursor(2, 0);
        lcd.print("Line");
        lcd.setCursor(3, 1);
        lcd.print(lineRead());
        break;
      case 1:
      //ロボットがどの区分にいるかを表示
        lcd.clear();
        lcd.backlight();
        lcd.setCursor(2, 0);
        lcd.print("Position");
        lcd.setCursor(3, 1);
        lcd.print("x:");
        lcd.print(x);
        lcd.setCursor(8, 1);
        lcd.print("y:");
        lcd.print(y);
        break;
      case 2:
      //方位センサの値を表示
        lcd.clear();
        lcd.backlight();
        lcd.setCursor(2, 0);
        lcd.print("Compass");
        lcd.setCursor(3, 1);
        lcd.print(e);//偏差
        lcd.print("deg");
        lcd.print(front);//基準
        lcd.print("deg");
        break;
      case 3:
      //irセンサを読むマイコンから送られてくる進行方向を表示
        lcd.clear();
        lcd.backlight();
        lcd.setCursor(2, 0);
        lcd.print("Direction");
        lcd.setCursor(3, 1);
        lcd.print(irRead());
        break;
    }
    delay(100);
  } else if (dir == 360) {//ボールがコートから出された場合
    move(mani, mani, mani);//その場で相手ゴール側に向く
  } else {
    switch (line) {
      //ラインセンサに反応があれば進行方向を変える
      case 1:
        dir = 330;
        break;
      case 2:
        dir = 210;
        break;
      case 3:
        dir = 270;
        break;
      case 4:
        dir = 90;
        break;
      case 5:
        dir = 30;
        break;
      case 6:
        dir = 150;
        break;
      default:
        break;
    }
    int pwm = 90;
    if(x%4>1){
      if((digitalRead(BALL)==LOW)&&(interval-millis()>500)){
      //キッカーがボールに届き,かつ充電が終わっていればキッカーを動かす
        digitalWrite(KICKER,HIGH);
        digitalWrite(KICKER,LOW);
        interval = millis();
      }
    }else if(
      ((x==0)&&(dir<45)&&(dir>315))//右端にいて右に進もうとしている場合
      ||((x==4)&&(dir>135)&&(dir<225))//左端にいて左に進もうとしている場合
      ||((y<2)&&(dir>45)&&(dir<135))//相手側のゴールライン付近で前に進もうとしてる場合
      ||((y==9)&&(y==8)&&(dir>225)&&(dir<315))//自陣側のゴールライン付近で後ろに進もうとしてる場合
    ){
      pwm = 64;
    }
    dir2out(dir, pwm, &m0, &m1, &m2);//進行方向からモタドラへの出力を決める
    //モタドラへの出力にpd制御の操作量を加える
    m0 += mani;
    m1 += mani;
    m2 += mani;
    move(m0, m1, m2);//モーターを動かす
  }
}
