
#pragma once

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <span>

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

  void stopRobot();
  void startRobot();

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

  //virtual void writeOnce(const std::array<double, 17>& joint_hardware_command);
  void writeJointPositions(const std::vector<double>& position);

 protected:
  Robot() = default;

 private:
  void receive();
  void send(const std::span<const uint8_t> & data);
  void get_position();
  void compute_hwval();

  std::mutex hw_lock;

  std::string device_name_;
  rclcpp::Logger logger_;
  int serial_device_;
  //LibSerial::SerialPort serial_device_;
  std::thread t;
  bool running_;

  static const int MAX_CHANNELS = 24;
  static const int REAL_CHANNELS = 17;
  static const int CORRECTION = -50;
  static const int COUNTS_PER_REV = 1200; // just guessing

  const std::array<uint8_t, 3> initialize = {250, 2, 7};
  const std::array<uint8_t, 4> enable = {254, 3, 2, 1};
  const std::array<uint8_t, 4> disable = {254, 3,2, 0};
  const std::array<uint8_t, 3> position = {0, 2, 3};
  // home_position is in 16 bit format
  const std::array<uint16_t, 24> home_position = {
    467,
    281,
    365,
    456,
    656,
    509,
    755,
    224,
    437,
    446,
    437,
    485,
    704,
    527,
    212,
    383,
    467,
    461,
    461,
    461,
    386,
    386,
    461,
    461,
  };
  enum State { IDLE, START1, START2, ADDRESS, DATA };

  std::array<uint16_t, MAX_CHANNELS> current_position = {0};
  std::array<uint8_t, MAX_CHANNELS * 2> hw_val = {0};
  int receive_count;
  float conversion;
};
}
