// --- Include the relvant libraries. ---
#include <MotorWheel.h>
#include <Omni3WD.h>
#include <Omni4WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>


// --- Define ISR's for motor control. ---
irqISR(irq1,isr1); // Intterrupt function.on the basis of the pulse,work for wheel1
MotorWheel wheel1(3,2,4,5,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);

Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4);


// --- Define variable tpyes ---
// Float variables for control (TODO: use list of floats instead)
float spd;
float dir;
float rot;

// Update freqency
int motor_delta_time = 20;
int serial_timeout = 10;

// Serial read buffer variables  (TODO: use list of floats and iterative sequency instead)
String incomingStr;
int del1_idx;
int del2_idx;

bool has_timeout = true;

// watchdogs
unsigned long max_time_between_updates = 250; //Evey 250ms a update must be recieved.
unsigned long time_since_last_update = millis(); 
unsigned long current_time = millis();


// --- Data functions ---
static void read_command_from_serial(){
  // (TODO: use list of floats and iterative sequency instead)
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingStr = Serial.readString();
    time_since_last_update = millis();

    del1_idx = incomingStr.indexOf(',');
    spd = incomingStr.substring(0,del1_idx).toFloat();
    String remain = incomingStr.substring(del1_idx+1, incomingStr.length()-1);

    del2_idx = remain.indexOf(',');
    dir = remain.substring(0,del2_idx).toFloat();
    rot = remain.substring(del2_idx+1, incomingStr.length()-1).toFloat();

    // DEBUG LED
    digitalWrite(LED_BUILTIN, HIGH);
//    //DEBUG
//    Serial.print(spd);
//    Serial.print(",");
//    Serial.print(dir);
//    Serial.print(",");
//    Serial.println(rot);
    
  }
}


bool timeout_overwrite(){
  current_time = millis();

  // in case no new data has been recieved for longer than max_time_between_updates
  if ((current_time - time_since_last_update) > max_time_between_updates){

    spd = 0;
    dir = 0;
    rot = 0;

    digitalWrite(LED_BUILTIN, LOW);

    return true
  return false

//    // DEBUG
//    Serial.print("Timeout: ");
//    Serial.print(time_since_last_update);
//    Serial.print(", ");
//    Serial.println(current_time);
    
  }
}


// ---Arduino Setup and Loop ---
void setup() {
  // Motor PWM on pins
  TCCR1B=TCCR1B&0xf8|0x01; // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01; // Pin3,Pin11 PWM 31250Hz
  Omni.PIDEnable(0.31,0.01,0,10); // Enable PID control for motors

  // Set the baud rate for the SoftwareSerial object
  Serial.begin(9600);
  Serial.setTimeout(serial_timeout); // Very Important for speed to set timeout properly

  // Allow for DEBUG LED
  pinMode(LED_BUILTIN, OUTPUT);
}


void loop() {
  // Set target variables
  read_command_from_serial();
  has_timeout = timeout_overwrite();

  // control motors based on variables
  if (has_timeout == true){
    Omni.setCarMove(spd, dir, rot);
    Omni.delayMS(motor_delta_time);
  } else{
    Omni.setCarStop()
  }
}