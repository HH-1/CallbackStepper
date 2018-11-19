// Author : HH-1 (https://github.com/HH-1)
// A Simple test for the CallbackStepper library
// A library to control stepper motor with 3 pins controller.
// Tested and develloped for TB-6600 controller and Arduino Uno 
// The sequence should be the following (assuming your motor doesn't loose any steps and has 200 steps/revolution : 
// Hold position for 3 seconds
// 10 revolutions at varying speed (increasing, decreasing)
// Hold position for 3 seconds
// Then repeat :
// free position for 3 seconds
// 100 revolutions at varying speed (increasing, stable, decreasing) in the other direction as previously
// free position for 3 seconds
// 10 revolutions at constant speed
// release free. 

#include "CallbackStepper.h"


// Define a motor with pins : 
//    ENA+ : 2
//    DIR+ : 3
//    STEP+ : 4
//    ENA- : GND
//    DIR- : GND
//    STEP- : GND
// The motor has 200 steps per revolution
// Minimum speed : 100 revolution per minute
// Maximum speed : 250 revolution per minute
// Acceleration : +20 revolution per minute per revolution
// At the beginning, the motor will be holding.
CallbackStepper motor1 = CallbackStepper(2,3,4,200, 100.0, 250.0, 20.0, true);

void setup() {
  Serial.begin(115200);
  delay(3000);
  motor1.runWithAcceleration(2000, printStepsPerformed, 200);
  delay(3000);
  motor1.stopHolding();

}
void loop() {
  delay(3000);
  motor1.runWithAcceleration(-20000, printStepsPerformed, 200);
  delay(1000);
  motor1.runWithoutAcceleration(2000, printStepsPerformed, 150);
}

void printStepsPerformed(int nbSteps){
  Serial.println(nbSteps);
}
