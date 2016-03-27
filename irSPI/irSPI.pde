#define DIVIDE 12.9//Adjusted to val/DEVIDE=90
#define THRESHOLD1 50
#define THRESHOLD2 800
#define LENGTH 15
#define ZZZZZ 3.5
//Mux control pins
int s0 = 11;
int s1 = 10;
int s2 = 9;
int s3 = 8;

int right[16]={
  22,21,20,19,18,17,16,15,28,29,30,27,26,25,1,2};
//Mux in "SIG" pin
int SIG_pin = 3;
int readMax(int channel){
  int controlPin[] = {
    s0, s1, s2, s3          };

  int muxChannel[16][4]={
    {
      1,0,1,0                    }
    , //channel 0
    {
      0,0,1,0                    }
    , //channel 1
    {
      1,1,0,0                    }
    , //channel 2
    {
      0,1,0,0                    }
    , //channel 3
    {
      1,0,0,0                    }
    , //channel 4
    {
      0,0,0,0                    }
    , //channel 5
    {
      1,1,1,0                    }
    , //channel 6
    {
      0,1,1,0                    }
    , //channel 7
    {
      1,0,0,1                    }
    , //channel 8
    {
      0,0,0,1                    }
    , //channel 9
    {
      1,1,1,1                    }
    , //channel 10
    {
      0,1,1,1                    }
    , //channel 11
    {
      1,0,1,1                    }
    , //channel 12
    {
      0,0,1,1                    }
    , //channel 13
    {
      1,1,0,1                    }
    , //channel 14
    {
      0,1,0,1                    }  //channel 15
  };

  //loop through the 4 sig
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the SIG pin
  int val = analogRead(SIG_pin);

  //return the value
  return val;
}
int goStraight(int val,int num){
  if((val==THRESHOLD1)&&(num==0)){
    return 360;
  }
  switch (num) {
  case 0:
    return 90;
  case 1:
    return 90;
  case 2:
    return 45;
  case 3:
    return 22.5;
  case 4:
    return 0;
  case 5:
    return 337.5;
  case 6:
    return 315;
  case 7:
    return 292.5;
  case 8:
    return 270;
  case 9:
    return 247.5;
  case 10:
    return 225;
  case 11:
    return 202.5;
  case 12:
    return 180;
  case 13:
    return 157.5;
  case 14:
    return 135;
  case 15:
    return 90;
  }
}
int gk(int val,int num){
  switch(num){
  case 0:
    if(val==THRESHOLD1){
      return 360;
    }
    if(readMax(2)>readMax(14)){
      return 0;
    }
    else{
      return 180;
    }
  case 1:
  case 2:
  case 3:
  case 4:
    return 0;
  case 12:
  case 13:
  case 14:
  case 15:
    return 180;
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
  case 10:
  case 11:
    return goAround(val,num);
  }
}
int goAround(int val,int num){
  if((val==THRESHOLD1)&&(num==0)){
    return 360;
  }
  double valiable = val/DIVIDE;
  if(valiable>90){
    valiable = 90;
  }
  int deg;
  switch (num) {
  case 0:
    deg=90;
    break;
  case 1:
    deg=90-valiable/ZZZZZ;
    break;
  case 2:
    if(valiable==90){
      deg = 361;
    }
    else{
      deg=45-valiable/2;
    }
    break;
  case 3:
    deg=22.5-valiable;
    break;
  case 4:
    deg=0-valiable;
    break;
  case 5:
    deg=-337.5-valiable;
    break;
  case 6:
    deg=315-valiable;
    break;
  case 7:
    deg=292.5-valiable;
    break;
  case 8:
    if(readMax(7)<readMax(9)){
      deg=270+valiable;
    }
    else{
      deg=270-valiable;
    }
    break;
  case 9:
    deg=247.5+valiable;
    break;
  case 10:
    deg=225+valiable;
    break;
  case 11:
    deg=202.5+valiable;
    break;
  case 12:
    deg=180+valiable/2;
    break;
  case 13:
    deg=157.5+valiable;
    break;
  case 14:
    if(valiable==90){
      deg = 362;
    }
    else{
      deg=135+valiable/2;
    }
    break;
  case 15:
    deg=90+valiable/ZZZZZ;
    break;
  }
  deg%=360;
  return deg;
}

int averaging(int* array){
  const int length = sizeof array/sizeof array[0];
  long sum=0;
  for (int i = 0;i < length;i++){
    if(array[i]>359){
      return array[i];
    }
    sum += array[i]%360;
  }
  return sum/length;
}

HardwareSPI spi(1);
uint8 recvd = 1;
void setup(){
  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(30, OUTPUT); 

  pinMode(7,INPUT_PULLUP);
  spi_init(SPI1);
  spi.beginSlave();
}

void loop(){
  //delay(50);
  int num=0,val=THRESHOLD1,temp,first=0;
  int difference[LENGTH]={
 0};
  long send=0;
  if((recvd != 1)||(val>THRESHOLD2)){
    for(int j = 0; j < LENGTH; j++){
      for(int i = 0; i < 16; i ++){
        temp=readMax(i);
        if(temp>val){
          val=temp;
          num=i;
        }
      }
      if(recvd == 2){
        difference[j]=goStraight(val,num);
      }
      else {
        difference[j]=goAround(val,num);
      }
      if(j==0){
        first=difference[j];
      }
      if(difference[j]<360){
        difference[j] -= first;
      }
      digitalWrite(right[num],HIGH);
      for(int k=0;k<16;k++){
        if(num==k){
          digitalWrite(right[k],HIGH);
        }
        else{
          digitalWrite(right[k],LOW);
        }
      }
    }
    send = averaging(difference);
  }
  else{
      for(int i = 0; i < 16; i ++){
        temp=readMax(i);
        if(temp>val){
          val=temp;
          num=i;
        }
      }
    send = gk(val,num);
    for(int k=0;k<16;k++){
        if(num==k){
          digitalWrite(right[k],HIGH);
        }
        else{
          digitalWrite(right[k],LOW);
        }
      }
  }
  if(send < 360){
    send += first;
  }
  SerialUSB.println(val);
  //  SerialUSB.print("dir");
  //  SerialUSB.println(send);
  if(send==360){
    send = 255;
  }
  else if(send == 361){
    SerialUSB.println("straight");
    send = 254;
  }
  else if(send == 362){
    SerialUSB.println("straight");
    send = 253;
  }
  else{
    send%=360;
    send*=7;
    send/=10;
  }
  spi_init(SPI1);
  spi.beginSlave();
  recvd = spi.transfer(send);
}


