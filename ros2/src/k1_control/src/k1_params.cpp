
#include "k1_control/k1_params.h"


namespace k1_control {

ParamServiceServer::ParamServiceServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot)
    : rclcpp::Node("service_server", options), robot_(std::move(robot)) 
{

  this->declare_parameter("foo", "bar");

  callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ParamServiceServer::setCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Service started");
}


rcl_interfaces::msg::SetParametersResult
ParamServiceServer::setCallback (const std::vector<rclcpp::Parameter> & parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
#if 0
  for (const auto & parameter : parameters) {
    if (!some_condition) {
      result.successful = false;
      result.reason = "the reason it could not be allowed";
    }
  }
#endif
  return result;
}

}
