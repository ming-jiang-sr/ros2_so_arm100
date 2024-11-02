from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveitpy_simple.moveit_configs_utils.file_loaders import load_xacro


def generate_launch_description():
    robot_description = load_xacro(
        get_package_share_path("so_arm100_description")
        / "urdf"
        / "so_arm100.urdf.xacro",
        mappings={"ros2_control_hardware_type": "mock_components"},
    )
    ros2_controllers_file = (
        get_package_share_path("so_arm100_description")
        / "control"
        / "ros2_controllers.yaml"
    )

    startup_controllers = [
        "joint_state_broadcaster",
        "joint_trajectory_controller",
        "gripper_controller",
    ]
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("so_arm100_description"),
                            "rviz",
                            "config.rviz",
                        ],
                    ),
                ],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description},
                    ros2_controllers_file,
                ],
                # To get logs from spdlog
                output="screen",
                # Colorful output
                emulate_tty=True,
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description},
                ],
            ),
        ]
        + [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
            )
            for controller in startup_controllers
        ],
    )
