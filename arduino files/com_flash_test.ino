#include <SoftwareSerial.h>

#define rxPin RX //10
#define txPin 1 //11

//SoftwareSerial RF_link = SoftwareSerial(rxPin, txPin); // RX/TX

String incomingStr;

void setup() {
  // put your setup code here, to run once:
//  pinMode(rxPin, INPUT);
//  pinMode(txPin, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
    
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

    
    // say what you got:
//    Serial.println(incomingStr);

    if (incomingStr == "1"){
      digitalWrite(LED_BUILTIN, HIGH);
      } else if (incomingStr == "0"){
      digitalWrite(LED_BUILTIN, LOW);
      }

  }
}

