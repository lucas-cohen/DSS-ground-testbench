// # include <SoftwareSerial.h>

#include <MotorWheel.h>
#include <Omni3WD.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#define rxPin 10 //10
#define txPin 11 //11



irqISR(irq1,isr1); // Intterrupt function.on the basis of the pulse,work for wheel1
MotorWheel wheel1(3,2,4,5,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);

Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4);


// SoftwareSerial RF_link = SoftwareSerial(rxPin, txPin); // RX/TX

float spd;
float dir;
float rot;

int dt = 50;

String incomingStr;
int del1_idx;
int del2_idx;

void setup() {
  // put your setup code here, to run once:
  // pinMode(rxPin, INPUT);
  // pinMode(txPin, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  
  TCCR1B=TCCR1B&0xf8|0x01; // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01; // Pin3,Pin11 PWM 31250Hz
  Omni.PIDEnable(0.31,0.01,0,10); // Enable PID

  
    
  // Set the baud rate for the SoftwareSerial object
  Serial.begin(9600);
  Serial.setTimeout(10); // Very Important for speed to set timeout properly
  
//  Serial.begin(9600);
//  Serial.setTimeout(10); 

  // Set baud rate of serial Comunication trough USB
  // Serial.begin(9600);
}

void loop() {
//  // write hello world
//  RF_link.println("hello world");
//  RF_link.flush();
//  delay(5);



  // Read data and write to USB serial
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingStr = Serial.readString();

    del1_idx = incomingStr.indexOf(',');
    spd = incomingStr.substring(0,del1_idx).toFloat();
    String remain = incomingStr.substring(del1_idx+1, incomingStr.length()-1);

    del2_idx = remain.indexOf(',');
    dir = remain.substring(0,del2_idx).toFloat();
    rot = remain.substring(del2_idx+1, incomingStr.length()-1).toFloat();
    

    
    // say what you got:

//    Serial.print(spd);
//    Serial.print(",");
//    Serial.print(dir);
//    Serial.print(",");
//    Serial.println(rot);
  }
  
  Omni.setCarMove(spd, dir, rot);
  Omni.delayMS(dt);
  
}