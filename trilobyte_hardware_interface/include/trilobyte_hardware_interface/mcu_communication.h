#ifndef MCU_COMMUNICATION_H
#define MCU_COMMUNICATION_H

#include <cstdint>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip> 

#include "rs232.h"

class MCUCommunication {

    public:

        MCUCommunication()
        {};

        MCUCommunication(uint8_t serial_device_id_number, uint32_t baud_rate, uint16_t timeout_ms);

        void moveForward(void);

        void sendMotorCommand(int16_t pwm_left_motor, int16_t pwm_right_motor);

    private:

        uint8_t serial_device_id_number = 24;
        uint32_t baud_rate = 115200;
        uint16_t timeout_ms = 2000;

        char MODE[4] = {'8','N','1',0};
        const uint8_t BUFFER_SIZE = 12;
        const char LEFT_MARKER = '<';
        const char RIGHT_MARKER = '>';

        std::string pwmToString(int16_t pwm_value);



};

#endif