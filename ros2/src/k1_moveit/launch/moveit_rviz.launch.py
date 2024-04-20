from moveit_configs_utils import MoveItConfigsBuilder
#  from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_moveit_rviz_launch(moveit_config):
    """Launch file for rviz"""
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": "/home/cmeyers/projects/flexo/warehouse_db.sqlite",
    }

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        warehouse_ros_config,
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("k1", package_name="k1_moveit").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
