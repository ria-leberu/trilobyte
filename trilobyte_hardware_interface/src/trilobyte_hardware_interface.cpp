#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trilobyte_hardware_interface/trilobyte_hardware_interface.hpp"

namespace trilobyte_hardware_interface
{
  // on_init (core method) - communication between robot hardware is setup 
  hardware_interface::CallbackReturn TrilobyteControlSystem::on_init(
  const hardware_interface::HardwareInfo& info) {

    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(
    rclcpp::get_logger("TrilobyteControlSystem"), "Starting Trilobyte Hardware Control . . . ");

    std::string& device_pico = info_.hardware_parameters["device_pico"];
    std::string& device_esp = info_.hardware_parameters["device_esp"];

    uint32_t baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    uint16_t timeout_ms = std::stoi(info_.hardware_parameters["timeout"]);

    _pico.configure(device_pico, baud_rate, timeout_ms);
    _esp.configure(device_esp, baud_rate, timeout_ms);

    return hardware_interface::CallbackReturn::SUCCESS;
  }
    

  // (core method) - returns a list of state interfaces
  std::vector<hardware_interface::StateInterface>
  TrilobyteControlSystem::export_state_interfaces() {

    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_position_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_velocity_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_position_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_velocity_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[2].name, hardware_interface::HW_IF_POSITION, &lifter_position_));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[3].name, hardware_interface::HW_IF_POSITION, &gripper_position_));

    // Hardware Interface needs to be changed to show load
    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    // info_.joints[2].name, hardware_interface::HW_IF_VELOCITY, &right_velocity_));


    return state_interfaces;
  }

  // (core method) - returns a list of command interfaces
  std::vector<hardware_interface::CommandInterface>
  TrilobyteControlSystem::export_command_interfaces() {

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_command_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_command_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[2].name, "position", &lifter_command_));
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[3].name, "position", &gripper_command_));

    return command_interfaces;
  }

  // read (core method) - updates the data values of the state_interfaces
  hardware_interface::return_type TrilobyteControlSystem::read(
  const rclcpp::Time& /*time*/, 
  const rclcpp::Duration& period) {

    std::array<int,2> output_encoder = _pico.read_encoder_values();
    std::array<int,4> output_servo_values = _esp.read_servo_values();

    int left_encoder_ticks = output_encoder[0];
    int right_encoder_ticks = output_encoder[1];

    double dt = period.seconds();

    left_position_ += (left_encoder_ticks - prev_left_encoder_ticks_) * (2 * PI_VALUE / 500);
    right_position_ += (right_encoder_ticks - prev_right_encoder_ticks_) * (2 * PI_VALUE / 500);

    left_velocity_ = ((left_encoder_ticks - prev_left_encoder_ticks_) * (2 * PI_VALUE / 500)) / dt;
    right_velocity_ = ((right_encoder_ticks - prev_right_encoder_ticks_) * (2 * PI_VALUE / 500)) / dt;

    // Store current encoder values for next cycle
    prev_left_encoder_ticks_ = left_encoder_ticks;
    prev_right_encoder_ticks_ = right_encoder_ticks;

    lifter_position_ = output_servo_values[0];
    gripper_position_ = output_servo_values[1];
    lifter_load_ = output_servo_values[2];
    gripper_load_ = output_servo_values[3];

    RCLCPP_DEBUG(
    rclcpp::get_logger("TrilobyteControlSystem"), 
    "lifter: %d gripper: %d", 
    lifter_position_, gripper_position_);

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type TrilobyteControlSystem::write(
  const rclcpp::Time& /*time*/, 
  const rclcpp::Duration& /*period*/)  {

    int16_t left_pwm = ((left_command_* 0.1885)/(2 * PI_VALUE) * 278);
    int16_t right_pwm = ((right_command_ * 0.1885)/(2 * PI_VALUE) * 278);

    double lifter_rads, gripper_rads;

    if (lifter_command_ > this->MAX_ANGLE_LIFTER) {
      lifter_rads = this->MAX_ANGLE_LIFTER;
        RCLCPP_INFO(
      rclcpp::get_logger("TrilobyteControlSystem"), 
      "Lifter at maximum angle.");
    } else if (lifter_command_ < this->MIN_ANGLE_LIFTER) {
      lifter_rads = this->MIN_ANGLE_LIFTER;
      RCLCPP_INFO(
      rclcpp::get_logger("TrilobyteControlSystem"), 
      "Lifter at minimum angle.");
    } else {
      lifter_rads = lifter_command_;
    }

    if (gripper_command_ > this->MAX_ANGLE_GRIPPER) {
      gripper_rads = this->MAX_ANGLE_GRIPPER;
      RCLCPP_INFO(
      rclcpp::get_logger("TrilobyteControlSystem"), 
      "Gripper at maximum angle.");
    } else if (gripper_command_ < this->MIN_ANGLE_GRIPPER) {
      gripper_rads = this->MIN_ANGLE_GRIPPER;
      RCLCPP_INFO(
      rclcpp::get_logger("TrilobyteControlSystem"), 
      "Gripper at minimum angle.");
    } else {
      gripper_rads = gripper_command_;
    }

    uint16_t lifter_pos = (lifter_rads * 652) + 2047;
    uint16_t gripper_pos = (gripper_rads * 652) + 2047;

    _pico.send_motor_command(left_pwm, right_pwm);



    _esp.send_servo_command(lifter_pos, gripper_pos);

    return hardware_interface::return_type::OK;
  }

} // namespace my_robot_hardware_interface


  #include "pluginlib/class_list_macros.hpp"

  PLUGINLIB_EXPORT_CLASS(
  trilobyte_hardware_interface::TrilobyteControlSystem, hardware_interface::SystemInterface)