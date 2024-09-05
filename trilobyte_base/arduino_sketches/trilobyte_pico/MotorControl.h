#ifndef MotorControl_h
#define MotorControl_h

class MotorControl
{

  uint8_t pin_enable;
  uint8_t pin_in_1;
  uint8_t pin_in_2;
  
  uint8_t speed_motor = 0;
  bool is_clockwise = true;
  
  public:

  MotorControl(void)
  {
    ;
  }

  void setupMotor(uint8_t uintEna, uint8_t uintIn1, uint8_t uintIn2)
  {
    this->pin_enable = uintEna;
    this->pin_in_1 = uintIn1;
    this->pin_in_2 = uintIn2;
    
    pinMode(pin_in_1,OUTPUT);
    pinMode(pin_in_2,OUTPUT);

    //Serial.println("Setup has occurred");
    
  }

  void setPWM(uint8_t uintPWM)
  {
    analogWrite(pin_enable, uintPWM);
  }

  bool setClockwise(void)
  {
    if(!this->is_clockwise)
    {
      this->is_clockwise = true;
    }
    return this->is_clockwise;
  }

  bool setCounterClockwise(void)
  {
    if(this->is_clockwise)
    {
      this->is_clockwise = false;
    }
    return this->is_clockwise;
  }

  void startMotor(void)
  {
    if(this->is_clockwise)
    {
      digitalWrite(pin_in_1, HIGH);
      digitalWrite(pin_in_2, LOW);
    }
    else
    {
      digitalWrite(pin_in_1, LOW);
      digitalWrite(pin_in_2, HIGH);
    }
  }

  void stopMotor(void)
  {
    setPWM(0);
  }
  
  
};

#endif