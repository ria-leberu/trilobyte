#ifndef MCU_COMMUNICATION_H
#define MCU_COMMUNICATION_H

#include <cstdint>
#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip> 
#include <chrono>
#include <array>

// #include "rs232.h"
#include "serial/serial.h"

#define PI_VALUE 3.1416


class MCUCommunication {

    public:

        MCUCommunication();

        ~MCUCommunication();

        void configure(std::string device_name, uint32_t baud_rate, uint16_t timeout_ms);

        void send_motor_command(int16_t pwm_left_motor, int16_t pwm_right_motor);

        std::array<int,2> read_encoder_values(void);

        void update_wheel_positions(void);

        std::chrono::time_point<std::chrono::system_clock> time_previous;
        double vel_wheel_left = 0.0;
        double vel_wheel_right = 0.0;
        double pos_wheel_left = 0.0;
        double pos_wheel_right = 0.0;

        uint32_t encoder_left = 0;
        uint32_t encoder_right = 0;


    private:

        std::string _serial_device = "/dev/ttyACM0";
        uint32_t _baud_rate = 115200;
        uint16_t _timeout_ms = 2000;
        uint16_t _count_per_revolution = 138;
        double _radians_per_count = (2 * PI_VALUE) / _count_per_revolution;

        const char LEFT_MARKER = '<';
        const char RIGHT_MARKER = '>';

        std::string _response;
        serial::Serial _serial_conn;

        std::string _convert_pwm_to_string(int16_t pwm_value);

};


#endif