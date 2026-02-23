#include "jetbot_hardware/jetbot_hardware_interface.hpp"

#include <cstring>
#include <sstream>
#include <iomanip>

// POSIX serial port headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jetbot_hardware
{

hardware_interface::CallbackReturn JetbotHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate joint count
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("JetbotHardwareInterface"),
      "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read URDF <param> values
  port_ = info_.hardware_parameters.count("port") ?
    info_.hardware_parameters.at("port") : "/dev/ttyTHS1";

  baud_rate_ = info_.hardware_parameters.count("baud_rate") ?
    std::stoi(info_.hardware_parameters.at("baud_rate")) : 115200;

  max_wheel_speed_rad_s_ = info_.hardware_parameters.count("max_wheel_speed_rad_s") ?
    std::stod(info_.hardware_parameters.at("max_wheel_speed_rad_s")) : 15.0;

  // Store joint names — first joint is left, second is right
  left_wheel_name_ = info_.joints[0].name;
  right_wheel_name_ = info_.joints[1].name;

  RCLCPP_INFO(
    rclcpp::get_logger("JetbotHardwareInterface"),
    "Initialized: port=%s baud=%d max_speed=%.1f rad/s",
    port_.c_str(), baud_rate_, max_wheel_speed_rad_s_);
  RCLCPP_INFO(
    rclcpp::get_logger("JetbotHardwareInterface"),
    "Left joint: %s  Right joint: %s",
    left_wheel_name_.c_str(), right_wheel_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetbotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_serial()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("JetbotHardwareInterface"),
      "Failed to open serial port %s", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("JetbotHardwareInterface"),
    "Serial port %s opened at %d baud", port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetbotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset state
  hw_positions_[0] = hw_positions_[1] = 0.0;
  hw_velocities_[0] = hw_velocities_[1] = 0.0;
  hw_commands_velocity_[0] = hw_commands_velocity_[1] = 0.0;

  // Safety: ensure motors are stopped before starting
  send_uart("L=0 R=0\n");

  RCLCPP_INFO(rclcpp::get_logger("JetbotHardwareInterface"), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetbotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop motors
  send_uart("L=0 R=0\n");
  RCLCPP_INFO(rclcpp::get_logger("JetbotHardwareInterface"), "Hardware deactivated, motors stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetbotHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial();
  RCLCPP_INFO(rclcpp::get_logger("JetbotHardwareInterface"), "Serial port closed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
JetbotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Left wheel: position + velocity
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name_, hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_wheel_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));

  // Right wheel: position + velocity
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name_, hardware_interface::HW_IF_POSITION, &hw_positions_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
JetbotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    left_wheel_name_, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[1]));

  return command_interfaces;
}

hardware_interface::return_type JetbotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Open-loop: echo commanded velocity as measured state
  // Position is integrated from velocity
  hw_velocities_[0] = hw_commands_velocity_[0];
  hw_velocities_[1] = hw_commands_velocity_[1];

  double dt = period.seconds();
  hw_positions_[0] += hw_velocities_[0] * dt;
  hw_positions_[1] += hw_velocities_[1] * dt;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JetbotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert rad/s → normalized [-1, 1]
  double left_norm = clamp(
    hw_commands_velocity_[0] / max_wheel_speed_rad_s_, -1.0, 1.0);
  double right_norm = clamp(
    hw_commands_velocity_[1] / max_wheel_speed_rad_s_, -1.0, 1.0);

  // Format: "L=x.xxx R=y.yyy\n"
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3)
      << "L=" << left_norm << " R=" << right_norm << "\n";

  if (!send_uart(oss.str())) {
    RCLCPP_WARN(
      rclcpp::get_logger("JetbotHardwareInterface"),
      "Failed to write to serial port");
  }

  return hardware_interface::return_type::OK;
}

bool JetbotHardwareInterface::open_serial()
{
  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("JetbotHardwareInterface"),
      "Cannot open %s: %s", port_.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("JetbotHardwareInterface"),
      "tcgetattr failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Set baud rate
  speed_t speed = B115200;
  if (baud_rate_ == 9600) {
    speed = B9600;
  } else if (baud_rate_ == 57600) {
    speed = B57600;
  } else if (baud_rate_ == 115200) {
    speed = B115200;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("JetbotHardwareInterface"),
      "Unsupported baud rate %d, defaulting to 115200", baud_rate_);
    speed = B115200;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1, no flow control, raw mode
  tty.c_cflag &= ~PARENB;   // No parity
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;        // 8 data bits
  tty.c_cflag &= ~CRTSCTS;  // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);           // No software flow control
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;    // Raw output

  tty.c_cc[VMIN] = 0;    // Non-blocking read
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("JetbotHardwareInterface"),
      "tcsetattr failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  return true;
}

void JetbotHardwareInterface::close_serial()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool JetbotHardwareInterface::send_uart(const std::string & msg)
{
  if (serial_fd_ < 0) {
    return false;
  }
  ssize_t bytes_written = ::write(serial_fd_, msg.c_str(), msg.size());
  return bytes_written == static_cast<ssize_t>(msg.size());
}

}  // namespace jetbot_hardware

PLUGINLIB_EXPORT_CLASS(
  jetbot_hardware::JetbotHardwareInterface,
  hardware_interface::SystemInterface)
