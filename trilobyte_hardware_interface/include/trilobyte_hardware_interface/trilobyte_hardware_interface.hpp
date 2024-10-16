#ifndef ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mcu_communication.h"

namespace trilobyte_hardware_interface
{
class TrilobyteControlSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrilobyteControlSystem)


  // Core Methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;


private:
  // Parameters for the RRBot simulation
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  double left_position_ = 0;
  double right_position_ = 0;
  double left_velocity_ = 0;
  double right_velocity_ = 0;


  double left_command_ = 0;
  double right_command_ = 0;

  std::string left_wheel_name_;
  std::string right_wheel_name_;
  uint16_t loop_rate_;
  std::string device_name_;
  uint16_t baud_rate_;
  uint16_t timeout_ms_;
  uint16_t pulses_per_meter_;
  uint16_t pulses_per_revolution_;
  double wheel_radius_;
  double wheel_separation_;




  MCUCommunication mcu;

  // Store the command for the simulated robot

};

}  // namespace trilobyte_hardware_interface

#endif 