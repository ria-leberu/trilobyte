
/**
*@file trilobyte_pico.ino
*
*@brief Software for Trilobyte Pico uC, to control motors, and receive data from Ultrasonic Sensors,
* IMU and Motor Encoders
* 
*@author ria
*jean@riabuildsthefuture.com
*/

#include <Adafruit_NeoPixel.h>
#include <MyDelay.h>
#include "MotorControl.h"


// NeoPixel Configuration
constexpr uint8_t NEOPIXEL_PIN = 16;
Adafruit_NeoPixel status_led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
uint8_t led_state = 0;
void statusIndicator(void);
MyDelay LEDStatus(500, statusIndicator);

// Robot Physical Configuration (in meters)
constexpr double WHEEL_RADIUS = 0.033;
constexpr double WHEEL_SEPARATION = 0.27;


/*
Motor Configuration

IN1/IN2/PWM = Result
L/H/H = CCW
H/L/H = CW
L/L/H = STOP
H/H/- = BREAK
*/


// Left Motor Configuration
constexpr uint8_t PWMA = 13;
constexpr uint8_t AIN1 = 12;
constexpr uint8_t AIN2 = 11;
constexpr uint8_t LEFT_PWM_LIMIT = 200;
constexpr bool LEFT_MOTOR_OFFSET = false;

// Right Motor Configuration
constexpr uint8_t PWMB = 10;
constexpr uint8_t BIN1 = 9;
constexpr uint8_t BIN2 = 8;
constexpr uint8_t RIGHT_PWM_LIMIT = 200;
constexpr bool RIGHT_MOTOR_OFFSET = true;

MotorControl LeftMotor(PWMA, AIN1, AIN2, LEFT_PWM_LIMIT, LEFT_MOTOR_OFFSET);
MotorControl RightMotor(PWMB, BIN1, BIN2, RIGHT_PWM_LIMIT, RIGHT_MOTOR_OFFSET);

// Motor Encoder Configuration
constexpr int16_t ENCODER_MIN = -32767;
constexpr int16_t ENCODER_MAX = 32768;

constexpr uint8_t LEFT_ENCODER = 20;
constexpr uint8_t LEFT_ENCODER_DIR = 1;

constexpr uint8_t RIGHT_ENCODER = 6;
constexpr uint8_t RIGHT_ENCODER_DIR = 0;

int16_t g_left_motor_count = 0;
int16_t g_right_motor_count = 0;

void countLeftEncoder(void);

/*
Serial Communication Configuration

Parsed Serial Message is a 10-char array
e.g. msg[] == f123b21300
msg[0] -> Direction of left motor (forward or backward)
msg[1] to msg[3] -> PWM for left motor
msg[4] -> Direction of right motor (forward or backward)
msg[5] to msg[7] -> PWM for right motor
msg[8] to msg[9] -> Status Code 
*/

constexpr uint8_t MSG_SIZE = 32;
char g_msg_rcv_chars[MSG_SIZE];
char g_msg_temp_chars[MSG_SIZE];
char g_msg_from_pc[MSG_SIZE];

bool g_new_data = false;

void rcvWithMarkers(void);
void outputDataToPC(void);
void processCommand(void);

void setup() {
  Serial.begin(115200);

  status_led.begin();
  // status_led.show();
  LEDStatus.start();

  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_DIR, INPUT);
  pinMode(RIGHT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_DIR, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), countLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), countRightEncoder, RISING);
}

void loop() {

  LEDStatus.update();

  rcvWithMarkers();
  processCommand();
  outputDataToPC();
}

void statusIndicator() {
  if (led_state == 0) {
    status_led.setBrightness(16);
    led_state = 1;
  } else {
    status_led.setBrightness(0);
    led_state = 0;
  }

  status_led.clear();
  status_led.setPixelColor(0, 255, 255, 0);

  status_led.show();

}

void rcvWithMarkers() {
  static boolean rcv_in_progress = false;
  static byte ndx = 0;
  char start_marker = '<';
  char end_marker = '>';
  char rc;

  while (Serial.available() > 0 && g_new_data == false) {
    rc = Serial.read();

    if (rcv_in_progress == true) {
      if (rc != end_marker) {
        g_msg_rcv_chars[ndx] = rc;
        ndx++;
        if (ndx >= MSG_SIZE) {
          ndx = MSG_SIZE - 1;
        }
      } else {
        g_msg_rcv_chars[ndx] = '\0';
        rcv_in_progress = false;
        ndx = 0;
        g_new_data = true;
      }
    } else if (rc == start_marker) {
      rcv_in_progress = true;
    }
  }
  return;
}

void outputDataToPC() {

  char buffer[14] = { 0 };

  sprintf(buffer, "<%06d%06d>", g_left_motor_count, g_right_motor_count);
  Serial.println(buffer);
  // Serial.print("<");
  // Serial.print(left_buffer);
  // Serial.print(" ");
  // Serial.print(right_buffer);
  // Serial.println(">");
  // Serial.print(g_left_motor_count);

  // Serial.print(g_right_motor_count);
}


void processCommand() {

  // Left Motor
  bool left_backward = false;
  bool right_backward = false;

  String left_pwm_parse = {};
  String right_pwm_parse = {};
  uint8_t left_pwm, right_pwm;

  if (g_new_data) {
    if (g_msg_rcv_chars[0] == 'b') {
      left_backward = true;
    }
    if (g_msg_rcv_chars[4] == 'b') {
      right_backward = true;
    }

    left_pwm_parse += g_msg_rcv_chars[1];
    left_pwm_parse += g_msg_rcv_chars[2];
    left_pwm_parse += g_msg_rcv_chars[3];

    right_pwm_parse += g_msg_rcv_chars[5];
    right_pwm_parse += g_msg_rcv_chars[6];
    right_pwm_parse += g_msg_rcv_chars[7];

    left_pwm = uint8_t(left_pwm_parse.toInt());
    right_pwm = uint8_t(right_pwm_parse.toInt());

    LeftMotor.setMotorMotion(left_pwm, left_backward);
    RightMotor.setMotorMotion(right_pwm, right_backward);

    g_new_data = false;
  }
}

void countLeftEncoder() {
  uint8_t val = digitalRead(LEFT_ENCODER_DIR);
  if (val > 0) {
    if (g_left_motor_count == ENCODER_MAX) {
      g_left_motor_count = ENCODER_MIN;
    } else {
      g_left_motor_count++;
    }
  } else {
    if (g_left_motor_count == ENCODER_MIN) {
      g_left_motor_count = ENCODER_MAX;
    } else {
      g_left_motor_count--;
    }
  }
}

void countRightEncoder() {
  uint8_t val = digitalRead(RIGHT_ENCODER_DIR);
  if (val > 0) {
    if (g_right_motor_count == ENCODER_MAX) {
      g_right_motor_count = ENCODER_MIN;
    } else {
      g_right_motor_count++;
    }
  } else {
    if (g_right_motor_count == ENCODER_MIN) {
      g_right_motor_count = ENCODER_MAX;
    } else {
      g_right_motor_count--;
    }
  }
}
