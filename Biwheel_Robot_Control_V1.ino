#include <RCReceive.h> // RCRecieve Libary. Enables reading of PWM signals from RC Rx. 
#include <debug.h>
#include <makros.h>

/* CHANGE VALUE TO CHANGE SCALING WITH DC MOTORS.
 *  
 *  If value is <1, then motors will have lower RPM/Volt. Inverse is true. 
 *  
 *  
 */
double SCALEFACTOR = 1;  

// Definitions Arduino pins connected to H-Bridge Driver
int enA = 10; // PWM Output
int enB = 11; // PWM Output
int IN1 = 4;  // Digital Output for Motor A
int IN2 = 5;  // Digital Output for Motor A
int IN3 = 6;  // Digital Output for Motor B
int IN4 = 7;  // Digital Output for Motor B

const byte PIN_RC_1 = 3; // Pin 3 of Arduino. (Left Motor Pin)
const byte PIN_RC_2 = 9; // Pin 9 of Arduino. (Right Motor Pin)
RCReceive rcReceiver1; //To Control Left Motor (Channel 1 on Tx Dx6)
RCReceive rcReceiver2; //To Control Right Motor (Channel 3 on Tx Dx6)
void setup() {
    Serial.begin(9600);
    rcReceiver1.attach(PIN_RC_1); // Attach Pins
    rcReceiver2.attach(PIN_RC_2);
    
    pinMode(LED_BUILTIN, OUTPUT); // Configuring LED for debugging.
    digitalWrite(LED_BUILTIN, LOW);
    
    rcReceiver1.getNP();
    
   // Declare outputs.
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
  rcReceiver1.poll();
  rcReceiver2.poll();
  // zero point determination?

  if (rcReceiver1.hasNP() || rcReceiver2.hasNP()) {
    doWork();
  }
  else if (rcReceiver1.hasError() && rcReceiver2.hasError()) {
    // run error function if no Tx is detected. 
    errorDetected();
  } 
}

void errorDetected() {
  // Blink LED.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  Serial.println("No Connection.");
}

// Controls Motor A. A Value of 123 of RCSpeed means netural Tx gimbal position. 
void MotorSpeed1(double RCSpeed) {
  RCSpeed = ( RCSpeed - 123 ) * SCALEFACTOR;
  //Serial.print("Motor Speed 2: ");
  //Serial.println(RCSpeed);
  if(RCSpeed > 0)
  {
   digitalWrite(IN1, HIGH);
   digitalWrite(IN2, LOW);

  analogWrite(enA, RCSpeed);

  }
  if(RCSpeed < 0)
  {
   digitalWrite(IN1, LOW);
   digitalWrite(IN2, HIGH);

  analogWrite(enA, abs(RCSpeed));
  }
}

// Controls Motor B.
void MotorSpeed2(double RCSpeed) {
  RCSpeed = ( RCSpeed - 123 ) * SCALEFACTOR;
  //Serial.print("Motor Speed 2: ");
  //Serial.println(RCSpeed);
  if(RCSpeed > 0)
  {
   digitalWrite(IN3, HIGH); 
   digitalWrite(IN4, LOW);
  analogWrite(enB, RCSpeed);
  }
  if(RCSpeed < 0)
  {

   digitalWrite(IN3, LOW); 
   digitalWrite(IN4, HIGH);
  analogWrite(enB, abs(RCSpeed));
  }
}

void doWork() {
  byte RCvalue1 = rcReceiver1.getValue();
  byte RCvalue2 = rcReceiver2.getValue();
  //Serial.println(rcReceiver1.getValue());
  delay(1); // Short time delay for motors to actually run
  MotorSpeed1(RCvalue1);
  MotorSpeed2(RCvalue2);
  // Debugging block
/*
  if (rcReceiver1.getValue() >= 208) { // Highest
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (rcReceiver1.getValue() <= 43) { // Lowest
    digitalWrite(LED_BUILTIN, LOW);
  }
  */
}
