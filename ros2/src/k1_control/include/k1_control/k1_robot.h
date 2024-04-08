
#pragma once

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <libserial/SerialPort.h>

#include <rclcpp/logger.hpp>
#include "k1_control/k1_robot.h"

namespace k1_control {

class Robot {
 public:
  /**
   * @param[in] robot_ip IP address or hostname of the robot.
   * @param[im] logger ROS Logger to print eventual warnings.
   */
  explicit Robot(const std::string& device);
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot& other) = delete;
  Robot& operator=(Robot&& other) = delete;
  Robot(Robot&& other) = delete;

  /// Stops the currently running loop and closes the connection with the robot.
  virtual ~Robot();

  /// Stops the continuous communication read with the connected robot
  virtual void stopRobot();

  /**
   * Get the current robot state
   * @return current robot state.
   */
  //virtual franka::RobotState readOnce();

  /**
   * Return pointer to the franka robot model object .
   * @return pointer to the current robot model.
   */
  //virtual franka_hardware::Model* getModel();

  virtual void writeOnce(const std::array<double, 17>& joint_hardware_command);

 protected:
  Robot() = default;

 private:
  /**
   * The robot will use set of positions until a different set of position are commanded.
   * @param[in] joint_position joint position command.
   */
  virtual void writeOnceJointPositions(const std::array<double, 17>& positions);
  void receive();

  std::mutex write_mutex_;
  std::mutex control_mutex_;

  rclcpp::Logger logger_;
  std::string device_name_;
  //int serial_device_;
  LibSerial::SerialPort serial_device_;
  std::thread t;
  bool running_;

  const int MAX_CHANNELS = 24;
  const int REAL_CHANNELS = 17;
  const int CORRECTION = -50;
};
}
