// Author : HH-1 (https://github.com/HH-1)
// A library to control stepper motor with 3 pins controller.
// Tested and develloped for TB-6600 controller and Arduino Uno Rev3

#pragma once

#include <Arduino.h>

class CallbackStepper{
  public:
    // Constructor. It is possible to run multiple motors from the same arduino. They must be controled throught 3 pins : enable(EN+), direction(DIR+) and step(STP+).
    // You have to provide the step number (characteristic of the motor), the minimum and maximum speed in rotations per minute and the acceleration (increase in speed at each revolution)
    // The last argument define if you want the motor to hold at start.
    CallbackStepper(int Pin_EN_5v, int Pin_DIR_5v, int Pin_STP_5v, int steps_per_Revolution = 200, double min_speed = 100.0, double max_speed = 250.0, double acc = 200.0, bool hold_on_start = false);
    
    // Those two functions allow you to run the motor with algebraic number of steps.
    // the callback is a function that will be executed every periodicity steps and at the end if steps is not an even number of periodicity. It's argument is the algebraic number of steps performed since the last call.
    // periodicity is the number of steps performed between each call of callback.
    //This function will run accelerating from the minimum speed to the maximum speed at the beginning. Then at max speed and finally between max speed and minimum speed.
    void runWithAcceleration(int steps, void callback(int), int periodicity=200);
    // This function will run the motor at maximum speed.
    void runWithoutAcceleration(int steps, void callback(int), int periodicity=200);
    
    double getMinSpeed();
    double getMaxSpeed();
    double getAcc();
    int getStepsPerRevolution();
    double setMinSpeed(double newSpeed);
    double setMaxSpeed(double newSpeed);
    double setAcc(double newAcc);
    int setStepsPerRevolution(int new_steps_per_revolution);
    
    // allow the motor to hold it's position.
    void startHolding();
    
    // disable holding position.
    void stopHolding();

  private:
    int _Pin_EN_5v;
    int _Pin_DIR_5v;
    int _Pin_STP_5v;
    double _min_speed;
    double _max_speed;
    double _acc;
    bool _holding;
    int _steps_per_revolution;
    double _StepsToAccelerate;
    // Compute the number of steps needed to go from min to max speed according to acceleration.
    int setStepsToAccelerate();
    
    void run(int steps, void callback(int), int periodicity, bool accelerate);

    // compute the speed for the next move.
    double computeActualSpeed(double actualSpeed, int absStepsToPerform, int i, int &accelerationPhase, double maxSpeedPossible);
    void setDirection(int direction);
    int getDirection(int nbStepsToPerform);
    void performOneStep(int stepDelay);
    int computeStepDelay(double speed_value);

};
