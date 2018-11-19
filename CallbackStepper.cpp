// Author : HH-1 (https://github.com/HH-1)
// A library to control stepper motor with 3 pins controller.
// Tested and develloped for TB-6600 controller and Arduino Uno Rev3


#include <Arduino.h>
#include "CallbackStepper.h"

//acc en tour.revolution.min-1 (cad min-1)
//speed en tour.min-1


CallbackStepper::CallbackStepper(int Pin_EN_5v, int Pin_DIR_5v, int Pin_STP_5v, int steps_per_Revolution, double min_speed, double max_speed, double acc, bool hold_on_start){
  _Pin_EN_5v = Pin_EN_5v;
  _Pin_DIR_5v = Pin_DIR_5v;
  _Pin_STP_5v = Pin_STP_5v;
  _steps_per_revolution = steps_per_Revolution;
  _min_speed = min_speed;
  _max_speed = max_speed;
  _acc = acc;
  _holding = hold_on_start;
  setStepsToAccelerate();
  pinMode(_Pin_EN_5v, OUTPUT);
  pinMode(_Pin_DIR_5v, OUTPUT);
  pinMode(_Pin_STP_5v, OUTPUT);
  if(hold_on_start){
    startHolding();
  }
  else{
    stopHolding();
  }
  digitalWrite (_Pin_DIR_5v, LOW); //DIR+(+5v)
  digitalWrite (_Pin_STP_5v, LOW); //PUL+(+5v)
}

int CallbackStepper::getStepsPerRevolution(){
  return _steps_per_revolution;
}

double CallbackStepper::getMinSpeed(){
  return _min_speed;
}

double CallbackStepper::getMaxSpeed(){
  return _max_speed;
}

double CallbackStepper::getAcc(){
  return _acc;
}

int CallbackStepper::setStepsPerRevolution(int new_steps_per_revolution){
  if(new_steps_per_revolution > 0){
    _steps_per_revolution = new_steps_per_revolution;
    setStepsToAccelerate();
  }
  return _steps_per_revolution;
}

double CallbackStepper::setMinSpeed(double newSpeed){
  if(newSpeed > 0){
    _min_speed = newSpeed;
    setStepsToAccelerate();  
  }
  return _min_speed;
}

double CallbackStepper::setMaxSpeed(double newSpeed){
  if(newSpeed > 0){
    _max_speed = newSpeed;
    setStepsToAccelerate();  
  }
  return _max_speed;
}
double CallbackStepper::setAcc(double newAcc){
  if(newAcc > 0){
    _acc = newAcc;
    setStepsToAccelerate();
  }
  return _acc;
}


int CallbackStepper::setStepsToAccelerate(){
  _StepsToAccelerate = 1 + (int) ((_max_speed-_min_speed)/_acc*_steps_per_revolution);
  return _StepsToAccelerate;
}


void CallbackStepper::startHolding(){
   digitalWrite (_Pin_EN_5v, LOW); //ENA+(+5V) low=enabled
   _holding = true;
}

void CallbackStepper::stopHolding(){
  digitalWrite (_Pin_EN_5v, HIGH);
  _holding = false;
}


void CallbackStepper::run(int steps, void callback(int), int periodicity, bool accelerate){
  bool wasHolding = _holding;
  if(!wasHolding){
    startHolding();
  }
  int direction = getDirection(steps);
  setDirection(direction);
  int absStepsToPerform = abs(steps);
  int max_speed_delay = computeStepDelay(_max_speed);
  
  if(accelerate){
    // accelerationPhase = 0,1 or -1 (0 accelerate, 1 constant speed, -1 decreasing speed)
    int accelerationPhase = 0;
    double actual_speed = _min_speed;
    double maxSpeedPossible = _StepsToAccelerate*2 > absStepsToPerform ? _min_speed + (double) (absStepsToPerform + 1)* _acc / (2 * _steps_per_revolution) : _max_speed;
    while(absStepsToPerform > periodicity){
      for(int i=0; i<periodicity; i++){
        actual_speed = computeActualSpeed(actual_speed, absStepsToPerform, i, accelerationPhase, maxSpeedPossible);
        performOneStep(actual_speed == _max_speed ? max_speed_delay : computeStepDelay(actual_speed));
      }
      absStepsToPerform -= periodicity;
      callback(direction*periodicity);
    }
    if (absStepsToPerform > 0){
      for(int i=0; i<absStepsToPerform; i++){
        actual_speed = computeActualSpeed(actual_speed, absStepsToPerform, i, accelerationPhase, maxSpeedPossible);
        performOneStep(actual_speed == _max_speed ? max_speed_delay : computeStepDelay(actual_speed));
      }
      callback(direction*absStepsToPerform);
    }
  }
  
  else{
    while(absStepsToPerform > periodicity){
      for(int i=0; i<periodicity; i++){
        performOneStep(max_speed_delay);
      }
      absStepsToPerform -= periodicity;
      callback(direction*periodicity);
    }
    if (absStepsToPerform > 0){
      for(int i=0; i<absStepsToPerform; i++){
        performOneStep(max_speed_delay);
      }
      callback(direction*absStepsToPerform);
    }
  }
  if(!wasHolding){
    stopHolding();
  }
}


void CallbackStepper::runWithAcceleration(int steps, void callback(int), int periodicity=200){
  run(steps, callback, periodicity, true);
}
void CallbackStepper::runWithoutAcceleration(int steps, void callback(int), int periodicity=200){
  run(steps, callback, periodicity, false);
}

double CallbackStepper::computeActualSpeed(double actualSpeed, int absStepsToPerform, int i, int &accelerationPhase, double maxSpeedPossible){
  if(accelerationPhase == 1 && absStepsToPerform-i == _StepsToAccelerate){
     accelerationPhase = -1;
  }
  if(accelerationPhase == 0){
    actualSpeed += _acc/_steps_per_revolution;
    if(actualSpeed >= maxSpeedPossible){
      actualSpeed = maxSpeedPossible;
      if(actualSpeed == _max_speed){
        accelerationPhase = 1;
      }
      else{
        accelerationPhase = -1;
      }
    }
  }
  else if(accelerationPhase == -1){
    if(actualSpeed != _min_speed){
      actualSpeed -= _acc/_steps_per_revolution;
      if (actualSpeed < _min_speed){
        actualSpeed = _min_speed;
      }
      else if (actualSpeed == _min_speed){
      }
    }
  }
  return actualSpeed;
}

void CallbackStepper::setDirection(int direction){
  if(direction > 0){
    digitalWrite (_Pin_DIR_5v,LOW);
  }
  else if(direction < 0){
    digitalWrite (_Pin_DIR_5v,HIGH);
    
  }
}

int CallbackStepper::getDirection(int nbStepsToPerform){
  if(nbStepsToPerform > 0){
    return 1;
  }
  else if(nbStepsToPerform == 0){
    return 0;
  }
  return -1;
}

void CallbackStepper::performOneStep(int stepDelay){
  // la vitesse est divisée par deux ?
  digitalWrite (_Pin_STP_5v, HIGH);
  delayMicroseconds (stepDelay);
  digitalWrite (_Pin_STP_5v, LOW);
}


int CallbackStepper::computeStepDelay(double speed_value){
  return (int) ((60.0/(float)((long)speed_value*(long)_steps_per_revolution))*1000000.0);
}
