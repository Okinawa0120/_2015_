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
#define INA0 24//移動用モーター
#define INB0 25//移動用モーター
#define PWM0 3//移動用モーター
#define INA1 23//移動用モーター
#define INB1 22//移動用モーター
#define PWM1 2//移動用モーター
#define INA2 28//移動用モーター
#define INB2 29//移動用モーター
#define PWM2 5//移動用モーター
#define INAD 26//ドリブラー
#define INBD 27//ドリブラー
#define PWMD 4//ドリブラー
#define SSIR 53//赤外線センサを読むマイコンのSSピン
#define SSUS 52//超音波センサを読むマイコンのSSピン
#define SSLN 43//ラインセンサを読むマイコンのSSピン
#define INTRPT 42//ライン割り込み(Interruptの略)
#define KICKER 41//ソレノイドを制御する電磁リレー
#define SWR 50//UI用のスイッチ
#define SWL 51//UI用のスイッチ
#define BT1 44//UI用のスイッチ
#define BT2 45//UI用のスイッチ
#define BT3 46//UI用のスイッチ
#define BALL 7//ボール検出センサ
//ピン番号以外のマクロ
#define NOM 5 //Number Of Modesの頭文字,モードの数
#define LPWM 90 //ラインの復帰スピード
#define IMAX 75
int pwm = 0;//移動する速さを調整する変数
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
int irRead(int send) {
  //irセンサを読むマイコンから進行方向を受け取る
  int recvd;
  digitalWrite(SSIR, LOW);
  recvd = SPI.transfer(send);
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
void move(double m0, double m1, double m2) {
  //移動用モーターを動かす
  if (m0 > 0) {
    digitalWrite(INA0, HIGH);
    digitalWrite(INB0, LOW);
    analogWrite(PWM0, m0);
  }
  else {
    m0 *= -1;
    digitalWrite(INA0, LOW);
    digitalWrite(INB0, HIGH);
    analogWrite(PWM0, m0);
  }

  if (m1 > 0) {
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);
    analogWrite(PWM1, m1);
  }
  else {
    m1 *= -1;
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
    analogWrite(PWM1, m1);
  }

  if (m2 > 0) {
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
    analogWrite(PWM2, m2);
  }
  else {
    m2 *= -1;
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
    analogWrite(PWM2, m2);
  }
}
void stop() {
  //移動用モーターを止める
  digitalWrite(INA0, LOW);
  digitalWrite(INB0, LOW);
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, LOW);
  digitalWrite(INA2, LOW);
  digitalWrite(INB2, LOW);
}
void dribble(int judg) {
  if (judg == 0) {
    digitalWrite(INAD, HIGH);
    digitalWrite(INBD, LOW);
    analogWrite(PWMD, 50);
  } else {
    digitalWrite(INAD, LOW);
    digitalWrite(INBD, LOW);
  }
}

int compassAddress = 0x42 >> 1;
int e = 0, e1 = 0, goal = 0,in=0;
double mani = 0, kd=10,kp=1.2,ki=0.42;
GC5883 compass;
void timerHandler() {
  e1=e;
  compass.init();
  e=goal-(int)compass;
  if(e>180){
    e-=360;
  }
  if(e<-180){
    e+=360;
  }
  in+=e;
  if(in>IMAX){
    in = IMAX;
  }else if(in<-IMAX){
    in = -IMAX;
  }
  if(pwm != 0){
    in = 0;
  }
  mani=0.6*kp*e+0.5*ki*in+0.125*kd*(e-e1);
  // 割り込み発生時に実行する部分
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
  e = (int)compass;
  Timer3.attachInterrupt(timerHandler).setFrequency(60).start();
}
int role = digitalRead(SWL); //ロボットのオフェンス,ディフェンスを表す変数
void change() {
  if (digitalRead(SWL) == LOW) {
    role = 0;//オフェンス
  } else {
    role = 1;//ディフェンス
  }
}
//------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
int front;
void setup() {
  //ioピンの設定
  pinMode(BT2,INPUT);
  pinMode(INA0, OUTPUT);
  pinMode(INB0, OUTPUT);
  pinMode(PWM0, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(SWR, INPUT);
  pinMode(SWL, INPUT);
  pinMode(BALL, INPUT);
  pinMode(KICKER, OUTPUT);
  digitalWrite(INA0, HIGH);
  digitalWrite(INB0, HIGH);
  digitalWrite(INA1, HIGH);
  digitalWrite(INB1, HIGH);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB2, HIGH);
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
  attachInterrupt(SWL, change, CHANGE);
}
int interval = 0;
void loop() {
  //SPI通信のSSピンをHIGHにする
  digitalWrite(SSLN, HIGH);
  digitalWrite(SSIR, HIGH);
  digitalWrite(SSUS, HIGH);
  //変数の初期化
  double m0 = 0, m1 = 0, m2 = 0;
  int dir = irRead(role);
  int line = lineRead();
  int y = posiRead() % 128;
  int x = y % 8;
  y /= 8;
  goal = front;
  if (digitalRead(SWR) == LOW) {
    //ロボット停止
    Timer3.stop();//I2C通信が中断されないようにタイマー割り込みを停止させる
    stop();//ロボットの動きを止める
    dribble(1);
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
        lcd.print(irRead(role));
        break;
      case 4:
        //ボールがキッカーの前にあるか表示
        lcd.clear();
        lcd.backlight();
        lcd.setCursor(2, 0);
        lcd.print("Ball");
        lcd.setCursor(3, 1);
        lcd.print(digitalRead(BALL));
        if(digitalRead(BT2)==HIGH) {
        //充電が終わっていればキッカーを動かす
        digitalWrite(KICKER, LOW);
        delay(10);
        digitalWrite(KICKER, HIGH);
        interval = millis();
      }
        break;
    }
    delay(100);
  } else {
    if (dir == 90) { //ボールが正面にあればドリブラーを回す
      dribble(0);
    } else {
      dribble(1);
    }
    pwm = 90;
    if (dir == 360) {//ボールがコートから出された場合
      if ((posiRead() / 128 == 1) ||
      (role == 0)) { //ゴール前にいるかオフェンスの時
        pwm = 0;//その場に止まる
      } else {
        //ゴール前に戻る
        if (x % 4 > 1) { //正面にゴールがあれば
          dir = 270;
          pwm = 70;
        } else {
          if (x / 4 == 0) { //コートの右半分にいれば
            dir = 180;
          } else {
            dir = 0;
          }
        }
      }
    }
    switch (line) {
        //ラインセンサに反応があれば進行方向を変える
      case 1:
        dir = 330;
        pwm = LPWM;
        break;
      case 2:
        dir = 210;
        pwm = LPWM;
        break;
      case 3:
        dir = 270;
        pwm = LPWM;
        break;
      case 4:
        dir = 90;
        pwm = LPWM;
        break;
      case 5:
        dir = 60;
        pwm = LPWM;
        break;
      case 6:
        dir = 120;
        pwm = LPWM;
        break;
      default:
        break;
    }
    if ((dir > 85) && (dir < 95) && digitalRead(BALL) == LOW) { //キッカーがボールに届くなら
      if (x % 4 <= 1) {//ゴールの正面にいない場合
        //x/4は左半分なら1右半分なら0
        //左半分にいるとき右に向き(+)右半分にいれば左に向く(-)
        int vari=0;
        if (y / 8 == 0) {
          vari = 20 - 2 * y;
        } else {
          vari = 10 + y;
        }
        if (x / 4 == 0) {
          goal = front + vari;
          e += vari;
          e1 = e;
        } else {
          goal = front - vari;
          e -= vari;
          e1 = e;
        }
        while ((e > -5) && (e < 5)) {
          move(0, mani/4, 0);
        }
      }else 
      if (interval - millis() > 5000) {
        //充電が終わっていればキッカーを動かす
        digitalWrite(KICKER, LOW);
        delay(5);
        digitalWrite(KICKER, HIGH);
        interval = millis();
      }
    }
//  if (//PD制御のパラメータが
//      ((x == 0) && (dir < 45) && (dir > 315)) //右端にいて右に進もうとしている場合
//      || ((x == 4) && (dir > 135) && (dir < 225)) //左端にいて左に進もうとしている場合
//      ||((x%4 == 0)//ゴールの正面にいない
//      && (
//        ((y < 2) && (dir > 45) && (dir < 135)) //相手側のゴールライン付近で前に進もうとしてる場合
//        || (((y == 9) || (y == 8)) && (dir > 225) && (dir < 315)) //自陣側のゴールライン付近で後ろに進もうとしてる場合
//      )
//    )
//    ) {
//      //ライン際では止まる
//      pwm = 0;
//    }
    if(x%4 == 0){
      if(f.mysin(dir)>0){
        if(y<2){
          dir = atan2(f.mysin(dir)/2,f.mycos(dir))*180/PI;
        }
      }else{
        if((y==9)||(y==8)){
          dir = atan2(f.mysin(dir)/2,f.mycos(dir))*180/PI;
      }
    }
    if(f.mycos(dir)>0){
      if(x==0){
        pwm *= f.mysin(dir);
        dir = atan2(1,0)*180/PI;
      }
    }else{
      if(x==4){
        pwm *= f.mysin(dir);
        dir = atan2(1,0)*180/PI;
      }
    }
  }
    dir2out(dir, pwm, &m0, &m1, &m2);//進行方向からモタドラへの出力を決める
    //モタドラへの出力にpd制御の操作量を加える
    m0 += mani;
    m1 += mani;
    m2 += mani;
    move(m0, m1, m2);//モーターを動かす
    
  }


}
