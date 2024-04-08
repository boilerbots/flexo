
#include <cassert>
#include <mutex>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>


#include <rclcpp/logging.hpp>

#include "k1_control/k1_robot.h"

using namespace LibSerial;

namespace k1_control {

Robot::Robot(const std::string& device) :
  device_name_(device),
  logger_(rclcpp::get_logger("k1_robot")),
  running_(false) 
{

#if 0
  serial_device_ = open(device_name_.c_str(), O_RDWR);
  // Set the serial port parameters
  termios tty;
  tcgetattr(serial_device_, &tty);
  tty.c_cflag = B115200;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CLOCAL;
  tty.c_cflag |= CREAD;
  tcsetattr(serial_device_, TCSANOW, &tty);
#endif
  RCLCPP_INFO_STREAM(logger_, "Opening: " << device_name_);
  serial_device_.Open(device_name_); // device_name_ doesn't work
  serial_device_.SetBaudRate( BaudRate::BAUD_115200 );

  t = std::thread(&Robot::receive, this);
}

Robot::~Robot() {
  stopRobot();
  running_ = false;
  //close(serial_device_);
  serial_device_.Close();
}

void Robot::receive()
{
  char next_byte;
  running_ = true;
  while(running_) {
    RCLCPP_INFO(logger_, "running");
    usleep(100000);
    //uint8_t readbuf = read(serial_device_, 1);
    try {
      serial_device_.ReadByte(next_byte, 1000);
    } catch (LibSerial::ReadTimeout) {
      ;
    }
  }
}

#if 0
k1_robot::RobotState Robot::readOnce() {
  std::lock_guard<std::mutex> lock(control_mutex_);
  if (!active_control_) {
    current_state_ = robot_->readOnce();
  } else {
    current_state_ = readOnceActiveControl();
  }
  return current_state_;
}
#endif

void Robot::stopRobot() {
}

void Robot::writeOnce(const std::array<double, 17>& joint_commands) {
  writeOnceJointPositions(joint_commands);
}


void Robot::writeOnceJointPositions(const std::array<double, 17>& positions) {
  std::lock_guard<std::mutex> lock(control_mutex_);

#if 0
  auto position_command = franka::JointPositions(positions);
  active_control_->writeOnce(position_command);
#endif
}

#if 0
k1_control::RobotState Robot::readOnceActiveControl() {
  // When controller is active use active control to read the robot state
  const auto [current_state, _] = active_control_->readOnce();
  return current_state;
}
#endif

}
