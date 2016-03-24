#include <JY901.h>
#include <movement.h>//自作ライブラリ
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
#define KICKER 7//ソレノイドを制御する電磁リレー
#define SWR 50//UI用のスイッチ
#define SWL 51//UI用のスイッチ
#define BT1 44//UI用のスイッチ
#define BT2 45//UI用のスイッチ
#define BT3 46//UI用のスイッチ
#define BALL 41//ボール検出センサ
//ピン番号以外のマクロ
#define NOM 5 //Number Of Modesの頭文字,モードの数
#define LPWM 100 //ラインの復帰スピード
#define IMAX 16
//------------------------------------------------
int posiRead() {
  //超音波センサを読むマイコンからロボットがどの区分にいるかを受け取る
  int recvd = 0;
  noInterrupts();
  digitalWrite(SSUS, LOW);
  recvd = SPI.transfer(0);//()の中の数に意味はない
  digitalWrite(SSUS, HIGH);
  interrupts();
  return recvd;
}
int lineRead() {
  //ラインセンサを読むマイコンからどのラインセンサが反応してるかを受け取る
  int recvd;
  noInterrupts();
  digitalWrite(SSLN, LOW);
  recvd = SPI.transfer(0);//()の中の数に意味はない
  digitalWrite(SSLN, HIGH);
  interrupts();
  return recvd;
}
int irRead(int send) {
  //irセンサを読むマイコンから進行方向を受け取る
  int recvd;
  noInterrupts();
  digitalWrite(SSIR, LOW);
  recvd = SPI.transfer(send);
  digitalWrite(SSIR, HIGH);
  interrupts();
  if (recvd == 255) {
    return 360;
  }
  if (recvd == 254) {
    return 361;
  }
  if (recvd == 253) {
    return 362;
  }
  recvd *= 10;
  recvd /= 7;
  return recvd;
}
mymath f;//sin,cosのテーブル参照の関数の自作クラス
movement m;//移動系の自作クラス
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

int e = 0, e1 = 0, goal = 0, in = 0;
double mani = 0, kd = 50, kp = 2.2, ki = 2;
int stopfg = 0; //停止フラグ
void timerHandler() {
  noInterrupts();
  e1 = e;
  JY901.GetAngle();
  e = goal - (float)JY901.stcAngle.Angle[2]/32768*180;
  if (e > 180) {
    e -= 360;
  }
  if (e < -180) {
    e += 360;
  }
  in += e;
  if (in > IMAX) {
    in = IMAX;
  } else if (in < -IMAX) {
    in = -IMAX;
  }
  if (stopfg == 0) {
    in = 0;
  }
  mani = 0.6 * kp * e + 0.5 * ki * in + 0.125 * kd * (e - e1);
  mani *= -1;
  m.setYaw(mani);
  // 割り込み発生時に実行する部分
  interrupts();
}
volatile int posi;
void timerHandler2(){
  posi = posiRead();
}

volatile int mode = 0;
volatile int sw = LOW;
volatile unsigned long time_prev = 0, time_now;
unsigned long time_chat = 200;
void increase() {
  //ボタン入力の処理
  time_now = millis();
  if( time_now-time_prev > time_chat){
    if( sw == LOW ){
      mode -= 1;
      if (mode < 0) {
        mode = NOM;
      }
    }
  }
    time_prev = time_now;
}
void decrease() {
  //ボタン入力の処理
    time_now = millis();
  if( time_now-time_prev > time_chat){
    if( sw == LOW ){
      mode += 1;
      if (mode >= NOM) {
        mode = 0;
      }
    }
  }
    time_prev = time_now;
}
void startTimer() {
  //スイッチ入力でタイマー割り込みを再開させるための関数
  JY901.GetAngle();
  e = goal - (float)JY901.stcAngle.Angle[2]/32768*180;
  Timer3.attachInterrupt(timerHandler).setFrequency(60).start();
  Timer4.attachInterrupt(timerHandler2).setFrequency(10).start();
}
int role; //ロボットのオフェンス,ディフェンスを表す変数
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
  pinMode(BT2, INPUT);
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
  SPI.setClockDivider(8);
  SPI.setDataMode(SPI_MODE0);
  Serial.begin(9600);
  pinMode(SSIR, OUTPUT);
  pinMode(SSUS, OUTPUT);
  pinMode(SSLN, OUTPUT);
  digitalWrite(SSIR, HIGH);
  digitalWrite(SSUS, HIGH);
  digitalWrite(SSLN, HIGH);
  //方位センサの設定
  digitalWrite(KICKER, LOW);
  delay(3000);
  JY901.StartIIC();
  JY901.GetAngle();
  front = (float)JY901.stcAngle.Angle[2]/32768*180;
  //タイマー割り込みの設定
  Timer3.attachInterrupt(timerHandler).setFrequency(60).start();
  Timer4.attachInterrupt(timerHandler2).setFrequency(10).start();
  //ピン割り込みの設定
  role = digitalRead(SWL);
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
  int pwm = 0;//移動する速さを調整する変数
  int y = posi % 128 / 8;
  int x = posi % 8;
  goal = front;
  if (digitalRead(SWR) == LOW) {
    //止まるとき
    Timer3.stop();
    Timer4.stop();//I2C通信が中断されないようにタイマー割り込みを停止させる
    m.stop();//ロボットの動きを止める
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
        posi = posiRead();
        y = posi % 128 / 8;
        x = posi % 8;
        lcd.clear();
        lcd.backlight();
        lcd.setCursor(2, 0);
        lcd.print("Position");
        lcd.print(posi / 128);
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
        lcd.print((float)JY901.stcAngle.Angle[2]/32768*180);//偏差
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
        if (digitalRead(BT2) == HIGH) {
          //充電が終わっていればキッカーを動かす
          digitalWrite(KICKER, HIGH);
          delay(50);
          digitalWrite(KICKER, LOW);
          interval = millis();
        }
        break;
    }
    delay(100);
  } else {//動くとき
    int dir = irRead(role);
    if (dir == 360) {//ボールがコートから出された場合
      if (posi / 128 == 1) { //ゴール前にいる時
        m.setX(0);
        m.setY(0);//その場に止まる
      } else {
        //ゴール前に戻る
        if (x % 4 > 2) { //正面にゴールがあれば
          if (role == 1) {
            m.setDir(270, 100); //後進する
          } else {
            m.setX(0);
            m.setY(0);//その場に止まる
          }
        } else {
          if (x / 4 == 0) { //コートの右半分にいれば
            m.setDir(180, 100);
          } else {
            m.setDir(0, 100);
          }
        }
      }
    } else {
      pwm = 165;
      if ((dir > 75) && (dir < 105)) { //ボールが正面にあればドリブラーを回す
        dribble(0);
      } else {
        dribble(1);
      }
      if (dir == 361) {
        dir = 0;
        pwm = 0;
      }
      if (dir == 362) {
        dir = 180;
        pwm = 0;
      }
      m.setDir(dir, pwm);
    }
    int line = lineRead();
    if (line != 0) {
      Serial.print("LINE");
      m.setY(LPWM * (line & 4) / 4 - LPWM * (line & 8) / 8);
      m.setX(LPWM * (line & 2) / 2 - LPWM * (line & 1));
      if (x == 0) {
      Serial.println("R");
        m.setX(- LPWM);
      }
      if (x == 4) {
        Serial.println("L");
        m.setX(LPWM);
      }
      
    }
    if ((dir > 85) && (dir < 95) && (digitalRead(BALL) == LOW)) { //キッカーがボールに届くなら
      if (x % 4 <= 1) {//ゴールの正面にいない場合
        //x/4は左半分なら1右半分なら0
        //左半分にいるとき右に向き(+)右半分にいれば左に向く(-)
        int vari = 0;
        if (y < 8) {
          vari = 50 - 2 * y;
        } else {
          vari = 30 + y;
        }
        if (x > 3) {
          goal = front + vari;
          if(goal > 180){
            goal -= 360;
          }
          e += vari;
          e1 = e;
        } else {
          goal = front - vari;
          if(goal < -180){
            goal += 360;
          }
          e -= vari;
          e1 = e;
        }
        while ((digitalRead(BALL) == LOW) && ((e < -5) || (e > 5))) {
          m.move(0.0, 0.0, mani);
        }
      }
      if (millis()-interval > 500) {
        //充電が終わっていればキッカーを動かす
        digitalWrite(KICKER, HIGH);
        delay(25);
        digitalWrite(KICKER, LOW);
        interval = millis();
      }
    }
    if ((x % 4 == 0) && (m.getY() > 0)) { //コートの左右どちらかの端にいて前進している場合
      if (y < 2) { //前方に障害物があれば
        m.setY(m.getY() / 2); //y成分を削減
      }
    }
    if ((m.getY() < 0) && (dir != 360)) { //後進していて
      if (y == 8) { //後方に障害物があれば
        irRead(2);
        m.setDir(irRead(role), pwm); //直線移動
        m.setY(0);
      }
    }
    if (m.getX() > 0) { //右に進んでいて
      if (x == 0) { //右端にいれば
        m.setX(0);//x成分を削除
      }
    } else { //左に進んでいて
      if (x == 4) { //左端にいれば
        m.setX(0);//x成分を削除
      }
    }
    if((role == 1)&&(y < 8)){
      m.setDir(270,100);
    }
    m.move();
    if ((m.getY() == 0) && (m.getX() == 0)) {
      stopfg = 1;
    } else {
      stopfg = 0;
    }
  }
}
