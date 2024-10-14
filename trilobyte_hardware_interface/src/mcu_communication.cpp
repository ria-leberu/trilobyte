#include "trilobyte_hardware_interface/mcu_communication.h"


MCUCommunication::MCUCommunication(uint8_t serial_device_id_number, uint32_t baud_rate, uint16_t timeout_ms) {

    this->serial_device_id_number = serial_device_id_number;
    this->baud_rate = baud_rate;
    this->timeout_ms = timeout_ms;

    return;
}

void MCUCommunication::sendMotorCommand(int16_t pwm_left_motor, int16_t pwm_right_motor) {
    // char buffer[10] = {0};

    // std::string buffer;

    std::string buffer = this->LEFT_MARKER + MCUCommunication::pwmToString(pwm_left_motor) +
    MCUCommunication::pwmToString(pwm_right_motor) + this->RIGHT_MARKER;

    std::cout << buffer << std::endl;


    if(RS232_OpenComport(this->serial_device_id_number, this->baud_rate, this->MODE, 0))
    {
    printf("Can not open comport\n");
    
    }

    // printf(buffer.c_str());
    // RCLCPP_INFO(
    // rclcpp::get_logger("TrilobyteControlSystem"), 
    // "String sent to MCU: %s", 
    // buffer.c_str());

    RS232_cputs(this->serial_device_id_number, buffer.c_str());

    RS232_CloseComport(this->serial_device_id_number);
    // buffer[0] = this->LEFT_MARKER;
    // buffer[9] = this->RIGHT_MARKER;

}

std::string MCUCommunication::pwmToString(int16_t pwm_value) {
    
    std::string pwm_command;
    std::string direction = "f";
    // char buffer[3] = {0};
    std::ostringstream oss;

    if (pwm_value < 0) {
        direction = "b";
        pwm_value = pwm_value * -1;
    }

    oss << std::setw(3) << std::setfill('0') << pwm_value;
    // sprintf(buffer, "%03d", pwm_value);
    pwm_command = direction + oss.str();

    // std::cout << pwm_command << "\n";

    return pwm_command;
}


// void MCUCommunication::moveForward(void) {

//       int cport_nr=24;
//   int bdrate=115200; /* 9600 baud */
 
//   char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
//   char str_send[3][128]; // send data buffer
// //   unsigned char str_recv[BUF_SIZE]; // recv data buffer
//   strcpy(str_send[0], "<f100f100>");
//   strcpy(str_send[1], "<b100b100>");
//   strcpy(str_send[2], "<f000f000>");
  
//   if(RS232_OpenComport(cport_nr, bdrate, mode, 0))
//   {
//     printf("Can not open comport\n");
    
//   }
 
//   usleep(200);  

//   RS232_cputs(cport_nr, str_send[0]);

//     usleep(2000000); 

//   RS232_cputs(cport_nr, str_send[1]);
//       usleep(2000000); 


// RS232_cputs(cport_nr, str_send[2]);

// }
