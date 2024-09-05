#ifndef PropulsionControl_h
#define PropulsionControl_h

// #include "Arduino.h"
#include "MotorControl.h"

class PropulsionControl
{

  MotorControl LeftMotor, RightMotor;
  // Left Drive


  // Right Drive
  
  
  public:

  PropulsionControl(void)
  {
    ;
  }

  void setupPropulsion(uint8_t uintENA, uint8_t uintIN1, uint8_t uintIN2, uint8_t uintENB, uint8_t uintIN3, uint8_t uintIN4)
  {
    LeftMotor.setupMotor(uintENA, uintIN1, uintIN2);
    RightMotor.setupMotor(uintENB, uintIN3, uintIN4);
  }

  void setLeftMotor(uint8_t uintPWM, bool bIsForward)
  {
    LeftMotor.setPWM(uintPWM);
    if(bIsForward)
    {
      LeftMotor.setClockwise();
    }
    else
    {
      LeftMotor.setCounterClockwise();
    }
    LeftMotor.startMotor();
  }

  void setRightMotor(uint8_t uintPWM, bool bIsForward)
  {
    RightMotor.setPWM(uintPWM);
    if(bIsForward)
    {
      RightMotor.setCounterClockwise();
    }
    else
    {
      RightMotor.setClockwise();
    }
    RightMotor.startMotor();
  }

  

  
};

#endif