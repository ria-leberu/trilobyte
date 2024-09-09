#ifndef MotorControl_h
#define MotorControl_h

class MotorControl {
  uint8_t pin_enable {};
  uint8_t pin_in1 {};
  uint8_t pin_in2 {};
  uint8_t pwm_limit {255};

  uint8_t motor_speed {0};
  bool is_offset {false};  // reverse the forward direction of rotation, use if the wiring is reversed


public:

  MotorControl(uint8_t enable, uint8_t in1, uint8_t in2, uint8_t pwm_limit, bool offset) {

    this->pin_enable = enable;
    this->is_offset = offset;

    if (!this->is_offset) {
      this->pin_in1 = in1;
      this->pin_in2 = in2;
    }
    else {
      this->pin_in1 = in2;
      this->pin_in2 = in1;
    }

    this->pwm_limit = pwm_limit;

    pinMode(this->pin_in1, OUTPUT);
    pinMode(this->pin_in2, OUTPUT);

    return;
  }

  void setMotorMotion(uint8_t pwm, bool is_backward) {

    if (!is_backward) {
      digitalWrite(this->pin_in1, HIGH);
      digitalWrite(this->pin_in2, LOW);
    } 
    else {
      digitalWrite(this->pin_in1, LOW);
      digitalWrite(this->pin_in2, HIGH);
    }

    if (pwm > this->pwm_limit)
    {
      pwm = this->pwm_limit;
    }

    analogWrite(this->pin_enable, pwm);

    return;
  }
};


#endif