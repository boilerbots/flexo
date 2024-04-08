#pragma once

#include <rclcpp/rclcpp.hpp>

namespace k1_control {

class Executor : public rclcpp::executors::MultiThreadedExecutor {
 public:
  // Create an instance and start the internal thread
  Executor();
  Executor(const Executor&) = delete;
  Executor(Executor&&) = delete;

  Executor& operator=(const Executor&) = delete;
  Executor& operator=(Executor&&) = delete;

  // Stops the internal executor and joins with the internal thread
  ~Executor() override;

 private:
  std::thread executor_spin_;

  // Executor thread starts spining the multithreadedExecutor
  void run();

  // Cancel any spinning ROS executor
  void shutdown();
};
}
