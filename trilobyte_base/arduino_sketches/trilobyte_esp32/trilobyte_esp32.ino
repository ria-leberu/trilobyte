/**
*@file trilobyte_esp32.ino
*
*@brief Software for Trilobyte ESP32 uC, to control servo motors.
* 
*@author ria
*jean@riabuildsthefuture.com
*/

#include <SCServo.h>

SMS_STS servo_controller;
constexpr uint8_t ESP32_RX = 18;
constexpr uint8_t ESP32_TX = 19;
constexpr uint16_t SERVO_CENTER_POSITION = 2047;
constexpr uint16_t SERVO_HALF_VELOCITY = 2047;

enum Servos {
  LIFTER,
  GRIPPER
};

constexpr uint8_t MSG_SIZE = 25;
char g_msg_rcv_chars[MSG_SIZE];
char g_msg_temp_chars[MSG_SIZE];
char g_msg_from_pc[MSG_SIZE];

bool g_new_data = false;

void rcvWithMarkers(void);
void outputDataToPC(void);
void processCommand(void);

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, ESP32_RX, ESP32_TX);
  servo_controller.pSerial = &Serial1;
  while(!Serial1) {}
  servo_controller.WritePosEx(LIFTER, SERVO_CENTER_POSITION, SERVO_HALF_VELOCITY);
  servo_controller.WritePosEx(GRIPPER, SERVO_CENTER_POSITION, SERVO_HALF_VELOCITY);
  delay(2000);
}

void loop() {

  rcvWithMarkers();
  processCommand();
  outputDataToPC();

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

  char buffer[26] = { 0 };
  uint16_t pos_lifter = 0;
  uint16_t pos_gripper = 0;
  uint16_t load_lifter = 0;
  uint16_t load_gripper = 0;

  if (servo_controller.FeedBack(0) != -1) {
    pos_lifter = servo_controller.ReadPos(-1);
    load_lifter = servo_controller.ReadLoad(-1);
  }
  if (servo_controller.FeedBack(1) != -1) {
    pos_gripper = servo_controller.ReadPos(-1);
    load_gripper = servo_controller.ReadLoad(-1);
  }

  sprintf(buffer, "<%05d%05d%05d%05d>", pos_lifter, pos_gripper, load_lifter, load_gripper);

  Serial.println(buffer);

}


void processCommand() {

  String lifter_position_parse = {};
  String gripper_position_parse = {};
  String lifter_speed_parse = {};
  String gripper_speed_parse = {};
  String lifter_acceleration_parse = {};
  String gripper_acceleration_parse = {};
  
  uint16_t lifter_position, gripper_position;
  uint16_t lifter_speed, gripper_speed;
  uint8_t lifter_acceleration, gripper_acceleration;

  if (g_new_data) {

    lifter_position_parse += g_msg_rcv_chars[0];
    lifter_position_parse += g_msg_rcv_chars[1];
    lifter_position_parse += g_msg_rcv_chars[2];
    lifter_position_parse += g_msg_rcv_chars[3];
    
    gripper_position_parse += g_msg_rcv_chars[4];
    gripper_position_parse += g_msg_rcv_chars[5];
    gripper_position_parse += g_msg_rcv_chars[6];
    gripper_position_parse += g_msg_rcv_chars[7];

    lifter_speed_parse += g_msg_rcv_chars[8];
    lifter_speed_parse += g_msg_rcv_chars[9];
    lifter_speed_parse += g_msg_rcv_chars[10];
    lifter_speed_parse += g_msg_rcv_chars[11];

    gripper_speed_parse += g_msg_rcv_chars[12];
    gripper_speed_parse += g_msg_rcv_chars[13];
    gripper_speed_parse += g_msg_rcv_chars[14];
    gripper_speed_parse += g_msg_rcv_chars[15];

    lifter_acceleration_parse += g_msg_rcv_chars[16];
    lifter_acceleration_parse += g_msg_rcv_chars[17];
    lifter_acceleration_parse += g_msg_rcv_chars[18];

    gripper_acceleration_parse += g_msg_rcv_chars[19];
    gripper_acceleration_parse += g_msg_rcv_chars[20];
    gripper_acceleration_parse += g_msg_rcv_chars[21];

    lifter_position = uint16_t(lifter_position_parse.toInt());
    gripper_position = uint16_t(gripper_position_parse.toInt());
    lifter_speed = uint16_t(lifter_speed_parse.toInt());
    gripper_speed = uint16_t(gripper_speed_parse.toInt());
    lifter_acceleration = uint8_t(lifter_acceleration_parse.toInt());
    gripper_acceleration = uint8_t(gripper_acceleration_parse.toInt());

    servo_controller.WritePosEx(LIFTER, lifter_position, lifter_speed);
    servo_controller.WritePosEx(GRIPPER, gripper_position, gripper_speed);

    g_new_data = false;
  }
}

