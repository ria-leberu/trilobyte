/**
*@file trilobyte_pico.ino
*
*@brief Software for Trilobyte Pico uC, to control motors, and receive data from Ultrasonic Sensors,
* IMU and Motor Encoders
* 
*@author ria
*jean@riabuildsthefuture.com
*/
#include <MPU6050.h>  // IMU Library
#include "MotorControl.h"

/*
IN1/IN2/PWM = Result
L/H/H = CCW
H/L/H = CW
L/L/H = STOP
H/H/- = BREAK
*/

// Left Motor Configuration
#define PWMA 2
#define AIN1 3
#define AIN2 4
#define LEFT_PWM_LIMIT 200
constexpr bool LEFT_MOTOR_OFFSET = false;

// Right Motor Configuration
#define PWMB 5
#define BIN1 6
#define BIN2 7
#define RIGHT_PWM_LIMIT 200
constexpr bool RIGHT_MOTOR_OFFSET = true;



MotorControl LeftMotor(PWMA, AIN1, AIN2, LEFT_PWM_LIMIT, LEFT_MOTOR_OFFSET);
MotorControl RightMotor(PWMB, BIN1, BIN2, RIGHT_PWM_LIMIT, RIGHT_MOTOR_OFFSET);



void setup() {
  Serial.begin(115200);
}

void loop() {

  LeftMotor.setMotorMotion(64, false);
  RightMotor.setMotorMotion(64, false);
}
