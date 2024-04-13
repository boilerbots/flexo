from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch
from launch_ros.actions import Node

#def generate_spawn_controllers_launch(moveit_config):
#    controller_names = moveit_config.trajectory_execution.get(
#        "moveit_simple_controller_manager", {}
#    ).get("controller_names", [])
#    ld = LaunchDescription()
#    for controller in controller_names + ["joint_state_broadcaster"]:
#        ld.add_action(
#            Node(
#                package="controller_manager",
#                executable="spawner",
#                arguments=[controller,
#                    "--controller-manager-timeout",
#                    "30",
#                ],
#                output="screen",
#            )
#        )
#    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("k1", package_name="k1_moveit").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
