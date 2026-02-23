#ifndef JETBOT_HARDWARE__JETBOT_HARDWARE_INTERFACE_HPP_
#define JETBOT_HARDWARE__JETBOT_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace jetbot_hardware
{

class JetbotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JetbotHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // UART serial port file descriptor
  int serial_fd_{-1};

  // Parameters from URDF <param> tags
  std::string port_;
  int baud_rate_;
  double max_wheel_speed_rad_s_;

  // Joint names (read from HardwareInfo)
  std::string left_wheel_name_;
  std::string right_wheel_name_;

  // State: position (integrated) and velocity for each wheel
  double hw_positions_[2]{0.0, 0.0};   // [left, right]
  double hw_velocities_[2]{0.0, 0.0};  // [left, right]

  // Commands: velocity in rad/s for each wheel
  double hw_commands_velocity_[2]{0.0, 0.0};  // [left, right]

  // Send a raw string over UART
  bool send_uart(const std::string & msg);

  // Open the serial port using POSIX termios
  bool open_serial();

  // Close the serial port
  void close_serial();

  // Clamp helper
  static double clamp(double value, double min_val, double max_val)
  {
    return value < min_val ? min_val : (value > max_val ? max_val : value);
  }
};

}  // namespace jetbot_hardware

#endif  // JETBOT_HARDWARE__JETBOT_HARDWARE_INTERFACE_HPP_
