#include<limits.h>
//#define fTRIG 21
//#define fECHO 22
//#define rTRIG 15
//#define rECHO 16
//#define lTRIG 29
//#define lECHO 30
//#define bTRIG 25
//#define bECHO 26
#define PENA 50
#define CENTER 75
int pin[8]={
  21,22,15,16,29,30,25,26};
HardwareSPI spi(1);
void setup(){
  //  pinMode(fTRIG,OUTPUT);
  //  pinMode(fECHO,INPUT);
  //  pinMode(rTRIG,OUTPUT);
  //  pinMode(rECHO,INPUT);
  //  pinMode(lTRIG,OUTPUT);
  //  pinMode(lECHO,INPUT);
  //  pinMode(bTRIG,OUTPUT);
  //  pinMode(bECHO,INPUT);
  for(int i=0;i<4;i++){
    pinMode(pin[2*i],OUTPUT);
    pinMode(pin[2*i+1],INPUT);
  }
  pinMode(7,INPUT_PULLUP);
  attachInterrupt(7, spitransfar, FALLING);
  spi_init(SPI1);
  spi.beginSlave();
}
float dist[4]={
  100,100,100,100};//前右左後
uint8 recvd;
uint8 send,x=0,y=0;
void loop() {
  uint8 goal = 0;
  for(int i=0;i<4;i++){
    float temp=0;
    SerialUSB.println(i);
    while(temp<=0.01){
      temp= distanceRead(pin[2*i],pin[2*i+1]);
    }
    dist[i] = temp;
  }
  SerialUSB.print("f:");
  SerialUSB.print(dist[0]);
  SerialUSB.print(" r:");
  SerialUSB.print(dist[1]);
  SerialUSB.print(" l:");
  SerialUSB.print(dist[2]);
  SerialUSB.print(" b:");
  SerialUSB.println(dist[3]);
  float xsum=dist[1]+dist[2];
  SerialUSB.print("x:");
  if((xsum>150)&&(xsum<190)){
    if(dist[1]>dist[2]){//right>left
      x=4;
      if(dist[2]>PENA){
        x+=(dist[2]-PENA)/13 + 1;
      }
    }
    else{
      if(dist[1]<PENA){
        x=0;
      }
      else{
        x=(dist[1]-PENA)/13 + 1;
      }

    } 
  }
  if(((dist[1]>CENTER)||(dist[2]>CENTER))&&(dist[3] < 10)){
    goal=128;
  }
  SerialUSB.println(x);
  delay(100);
  if(dist[0]>dist[3]){
    y=dist[3]/30+8;//30=distance of a block

  }
  else{
    y=dist[0]/30;//30=distance of a block
  }
  send=goal+x+y*8;
}
void spitransfar(){
  spi_init(SPI1);
  spi.beginSlave();
  recvd = spi.transfer(send);
  SerialUSB.println("!");
}
float distanceRead(int Trig,int Echo){
  int Duration;
  float Distance=0;
  digitalWrite(Trig,LOW);
  delayMicroseconds(1);
  digitalWrite(Trig,HIGH);
  delayMicroseconds(1);
  digitalWrite(Trig,LOW);
  Duration = pulseIn(Echo,HIGH,100);
  if (Duration>0) {
    Distance = Duration/2;
    Distance = Distance*340*100/1000000; // ultrasonic speed is 340m/s = 34000cm/s = 0.034cm/us 
  }
  return Distance;
}
unsigned long pulseIn(int iPin, int iValue, unsigned long ulTimeout)
{
  //
  // start and end times
  //

  unsigned long ulStartTime = 0;
  unsigned long ulEndTime = 0;

  //
  // start the timer
  //

  ulStartTime = micros();

  //
  // clear out the non desired reads to find the start of the pulese (LOW if iValue is HIGH, HIGH if iValue is LOW)
  //

  while (LOW == iValue ? HIGH == digitalRead(iPin) : LOW == digitalRead(iPin))
  {
    //
    // with each cycle, see if we have waited longer then the designated timeout
    //

    ulEndTime = micros();

    if (_Duration(ulStartTime, ulEndTime) > ulTimeout)
    {
      //
      // in the case of a timeout, the duration needs to be 0
      //

      ulStartTime = 0;
      ulEndTime = 0;

      //
      // this was a timeout expected error, so end the function
      //

      goto CleanUp;
    }
  }

  //
  // start the timer
  //

  ulStartTime = micros();

  //
  // keep looping until the recieve pin is not high anymore
  //

  while (iValue == digitalRead(iPin))
  {
    //
    // no code here
    //
  }

  //
  // get the amount of time the recieve pin was the expected value
  //

  ulEndTime = micros();

CleanUp:

  //
  // return the pulse length duration, compensating for timer overflow
  //

  return _Duration(ulStartTime, ulEndTime);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

unsigned long _Duration(unsigned long ulStartTime, unsigned long ulEndTime)
{
  unsigned long ulCurrentTime = 0;
  unsigned long ulDuration = 0;

  //
  // see if the there was a timer overflow
  //

  if (ulEndTime < ulStartTime)
  {
    //
    // there was a timer overflow, so compensate
    //

    ulCurrentTime = ULONG_MAX - ulStartTime;
    ulDuration = ulEndTime + ulCurrentTime;
  }
  else
  {
    //
    // calculate the final duration 
    //

    ulDuration = ulEndTime - ulStartTime;
  }

  //
  // return the calculated duration
  //

  return ulDuration;
}





