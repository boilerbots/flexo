
#pragma once

#include <memory>

#include "k1_control/k1_robot.h"
#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace k1_control {

class ParamServiceServer : public rclcpp::Node {
 public:
  ParamServiceServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot);

 private:
  rcl_interfaces::msg::SetParametersResult
  setCallback (const std::vector<rclcpp::Parameter> & parameters);

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  std::shared_ptr<Robot> robot_;

};
}
