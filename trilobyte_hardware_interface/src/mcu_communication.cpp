#include "trilobyte_hardware_interface/mcu_communication.h"


MCUCommunication::MCUCommunication() {

    return;
}

MCUCommunication::~MCUCommunication() {

    _serial_conn.close();
}

void MCUCommunication::configure(std::string serial_device_, uint32_t baud_rate, uint16_t timeout_ms) {
    this->_serial_conn.setPort(serial_device_);
    this->_serial_conn.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    this->_serial_conn.setTimeout(tt);
    this->_serial_conn.open();
    
}

void MCUCommunication::send_motor_command(int16_t pwm_left_motor, int16_t pwm_right_motor) {

    std::string buffer = this->LEFT_MARKER + MCUCommunication::_convert_pwm_to_string(pwm_left_motor) +
    MCUCommunication::_convert_pwm_to_string(pwm_right_motor) + this->RIGHT_MARKER;

    _serial_conn.flushOutput();
    _serial_conn.write(buffer);

}

std::array<int,2> MCUCommunication::read_encoder_values(void) {

    std::array<int,2> output_encoder;

    _serial_conn.flushInput();

    if (_serial_conn.readline().size() > 0) {
        this->_response = _serial_conn.readline();
    }

    std::string left_enc_str = this->_response.substr(1,6);
    std::string right_enc_str = this->_response.substr(7,6);

    output_encoder[0] = std::atoi(left_enc_str.c_str());
    output_encoder[1] = std::atoi(right_enc_str.c_str());

    return output_encoder;

}

std::string MCUCommunication::_convert_pwm_to_string(int16_t pwm_value) {
    
    std::string pwm_command;
    std::string direction = "f";
    std::ostringstream oss;

    if (pwm_value < 0) {
        direction = "b";
        pwm_value = pwm_value * -1;
    }

    oss << std::setw(3) << std::setfill('0') << pwm_value;
    pwm_command = direction + oss.str();

    return pwm_command;
}

void MCUCommunication::update_wheel_positions(void) {

    this->pos_wheel_left = this->encoder_left * this->_radians_per_count;
    this->pos_wheel_right = this->encoder_right * this->_radians_per_count;


}

