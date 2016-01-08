#include <GC5883.h>
#include <DueTimer.h>
/*
ライン
0反応なし
1左
2右
3左と右
4後ろ
5左と後ろ
6右と後ろ
*/
#include <mymath.h>
#include <Wire.h>
#include <SPI.h>
#define InA0 24
#define InB0 25
#define PWM0 3
#define InA1 23
#define InB1 22
#define PWM1 2
#define InA2 28
#define InB2 29
#define PWM2 5
#define SSIR 53//spi赤外せん
#define SSUS 52//spi超音波
#define SSLN 43//spiライン
#define INTRPT 42//ライン割り込み(Interruptの略)
#define KU 1
#define TU 4.5
#define SWR 51
#define SWL 50
int compassAddress = 0x42 >> 1;
int e = 0, e1 = 0, in = 0, head1;
double mani = 0, kd = TU, kp = KU;
GC5883 compass;
mymath f;
//------------------------------------------------
int lineRead() {
  int line;
  digitalWrite(SSLN, LOW);
  line = SPI.transfer(17);
  digitalWrite(SSLN, HIGH);
  return line;
}
int dirRead() {
  int dir;
  digitalWrite(SSIR, LOW);
  dir = SPI.transfer(9);

  digitalWrite(SSIR, HIGH);
  if (dir == 255) {
    return 360;
  }
  dir *= 10;
  dir /= 7;
  return dir;
}
void dir2out(int dir, int pwm, double *Mo0, double *Mo1, double *Mo2) {
  double x, y;
  x = pwm * f.mycos(dir);
  y = pwm * f.mysin(dir);
  *Mo0 = x * f.mycos(300) + y * f.mysin(300);
  *Mo1 = -1 * x;
  *Mo2 = x * f.mycos(60) + y * f.mysin(60);
}
void moveMotor(double Mo0, double Mo1, double Mo2) {
  if (Mo0 > 0) {
    digitalWrite(InA0, HIGH);
    digitalWrite(InB0, LOW);
    analogWrite(PWM0, Mo0);
  }
  else {
    Mo0 *= -1;
    digitalWrite(InA0, LOW);
    digitalWrite(InB0, HIGH);
    analogWrite(PWM0, Mo0);
  }

  if (Mo1 > 0) {
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    analogWrite(PWM1, Mo1);
  }
  else {
    Mo1 *= -1;
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    analogWrite(PWM1, Mo1);
  }

  if (Mo2 > 0) {
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);
    analogWrite(PWM2, Mo2);
  }
  else {
    Mo2 *= -1;
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, HIGH);
    analogWrite(PWM2, Mo2);
  }
}
void timerHandler() {
  e1 = e;
  compass.init();
  e = head1 - (int)compass;
  if (e > 180) {
    e -= 360;
  }
  if (e < -180) {
    e += 360;
  }
  in += e + e1;
  if ((e > -40) && (e < 40)) {
    kd = TU;
    kp = KU + 0.8;
  } else {
    kd = TU;
    kp = KU;
  }
  mani = 0.6 * kp * e + 0 * in + 0.125 * TU * (e - e1);
  // 割り込み発生時に実行する部分
}
//------------------------------------------------
void setup() {
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
  compass.init();
  head1 = (int)compass;
  Timer3.attachInterrupt(timerHandler).setFrequency(60).start();
}
void loop() {
  digitalWrite(SSLN, HIGH);
  digitalWrite(SSIR, HIGH);
  digitalWrite(SSUS, HIGH);
  int dir;
  double m0 = 0, m1 = 0, m2 = 0;
  dir = dirRead();
  Serial.print(dir);
  Serial.print(" ");
  Serial.println(lineRead());
  switch (lineRead()) {
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
  //  
  if ((dir == 360)||(digitalRead(SWR)==LOW)) {
    digitalWrite(InA0, LOW);
    digitalWrite(InB0, LOW);
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, LOW);
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, LOW);
  } else {
    dir2out(dir, 90, &m0, &m1, &m2);
    m0 += mani;
    m1 += mani;
    m2 += mani;
    moveMotor(m0, m1, m2);
  }
}
