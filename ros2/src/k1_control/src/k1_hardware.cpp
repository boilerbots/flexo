
#include "rclcpp/rclcpp.hpp"
#include "k1_control/k1_hardware.h"
#include <string>
#include <vector>
#include "k1_interfaces/msg/joints.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>


namespace k1_control
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  rclcpp::Logger logger = rclcpp::get_logger("k1_hardware");
  //lifecycle_node->get_node_parameters_interface();
  //get_node()->declare_parameter("my_parameter", "world");
  std::string serial_device =info_.hardware_parameters.at("device");
  //std::string serial_device("/dev/ttyAMA0");
  RCLCPP_INFO(logger, "Device: %s", serial_device.c_str());

  std::string servo_config_file("servo_config.yaml");
  std::string pose_file("poses.yaml");


  // robot has 17 joints
  joint_position_.assign(17, 0);
  joint_velocities_.assign(17, 0);
  joint_position_command_.assign(17, 0);
  joint_velocities_command_.assign(17, 0);

  // force sensor has 6 readings
  ft_states_.assign(6, 0);
  ft_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      //RCLCPP_INFO(logger, "Interface: %s  Joint %s", interface.name.c_str(), joint.name.c_str());
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  robot_ = std::make_shared<Robot>(serial_device);
  node_ = std::make_shared<ParamServiceServer>(rclcpp::NodeOptions(), robot_);
  executor_ = std::make_shared<Executor>();
  executor_->add_node(node_);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
#if 0
  rclcpp::Logger logger = rclcpp::get_logger("k1_hardware");

  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_[i] = joint_velocities_command_[i];
    joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  }
#endif

#if 0
  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    //RCLCPP_INFO(logger, "%d position %lf", i, joint_position_command_[i]);
    joint_position_[i] = joint_position_command_[i];
    //joint_position_[i] = i * 0.1;
  }
#endif
  robot_->readJointPositions(joint_position_);

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  robot_->writeJointPositions(joint_position_command_);
#if 0
  rclcpp::Logger logger = rclcpp::get_logger("k1_hardware");
  RCLCPP_INFO(logger, "#######");
  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    RCLCPP_INFO(logger, "  %lf", joint_position_command_[i]);
  }
#endif

  return return_type::OK;
}

#if 1
return_type RobotSystem::perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
{
  rclcpp::Logger logger = rclcpp::get_logger("k1_hardware");
  RCLCPP_INFO(logger, "\n\n Starting Robot \n\n");
  robot_->startRobot();

  robot_->readJointPositions(joint_position_);
  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    //RCLCPP_INFO(logger, "%d position %lf", i, joint_position_command_[i]);
    joint_position_command_[i] = joint_position_[i];
  }

  return return_type::OK;
}
#endif

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(k1_control::RobotSystem, hardware_interface::SystemInterface)
