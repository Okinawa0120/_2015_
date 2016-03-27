#define FRONT digitalRead(11)||digitalRead(12)||digitalRead(13)||digitalRead(14)||digitalRead(15)||digitalRead(16)
#define RIGHT digitalRead(0)||digitalRead(1)||digitalRead(2)||digitalRead(8)||digitalRead(9)||digitalRead(10)
#define LEFT  digitalRead(26)||digitalRead(27)||digitalRead(28)||digitalRead(29)||digitalRead(30)||digitalRead(31)
#define BACK  digitalRead(18)||digitalRead(19)||digitalRead(20)||digitalRead(21)||digitalRead(22)||digitalRead(25)
#define WAITIME 3000
HardwareSPI spi(1);
volatile uint8 recvd = 1,line=0;
int time1=0,time0=0,waitime;
short newresponse[2];//0:x 1:y

void setup() {
  pinMode(3, OUTPUT);
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(28, INPUT);
  pinMode(29, INPUT);
  pinMode(30, INPUT);
  pinMode(31, INPUT);

  pinMode(7,INPUT_PULLUP);
  attachInterrupt(7, spitransfar, FALLING);
  spi_init(SPI1);
  spi.beginSlave();
}
void spitransfar(){
  spi_init(SPI1);
  spi.beginSlave();
  recvd = spi.transfer(line);
  SerialUSB.println(line);
}
void loop() {
  int resfg = 0;
  if((millis()-time1>WAITIME)&&(millis()-time0>WAITIME)){
    waitime = 500;
  }
  if((LEFT)||(RIGHT)||(BACK)||(FRONT)){
    resfg = 1;
    time0 = millis();
//    time1 = millis();
  }

  //ライン処理-------------------------------------------
  if((LEFT)&&(newresponse[0]==0)){
    newresponse[0] = 1;
    time0 = millis();
    waitime = 100;
  }
  if((RIGHT)&&(newresponse[0]==0)){
    newresponse[0] = 2;
    time0 = millis();
    waitime = 100;
  }
  if((BACK)&&(newresponse[0]==0)){
    newresponse[0] = 4;
    time0 = millis();
    waitime = 100;
  }
  if((FRONT)&&(newresponse[0]==0)){
    newresponse[0] = 8;
    time0 = millis();
    waitime = 100;
  }
  if((millis()-time0>waitime)&&(resfg == 0)){
    newresponse[0] = 0;
  }
//  if((millis()-time1>waitime)&&(resfg == 0)){
//    newresponse[1] = 0;
//  }
  line = newresponse[0] ;//+ newresponse[1];
  //----------------------------------------------------
  if(line!=0){
    digitalWrite(3,HIGH);
  }
  else{
    digitalWrite(3,LOW);
  }
}

