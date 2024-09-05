/**
*@file trilobyte_pico.ino
*
*@brief Software for Trilobyte Pico uC, to control motors, and receive data from Ultrasonic Sensors,
* IMU and Motor Encoders
* 
*@author ria
*jean@riabuildsthefuture.com
*/
#include <MPU6050.h> // IMU Library
#include "PropulsionControl.h"

/*
IN1/IN2/PWM = Result
L/H/H = CCW
H/L/H = CW
L/L/H = STOP
H/H/- = BREAK
*/

// Left Motor Inputs
#define PWMA 2
#define AIN1 3
#define AIN2 4

// Right Motor Inputs
#define PWMB 5
#define BIN1 6
#define BIN2 7


void setup() {
  Serial.begin(115200);
}

void loop() {
}
