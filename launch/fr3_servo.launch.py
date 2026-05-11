import launch_ros
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

'''
作者：Daohui Liu
邮箱：daohui.liu@mail.utoronto.ca
@2025-07-22

本launch文件用于启动伺服节点

move_group和rviz2由fr3_moveit.launch.py启动，
robot_state_publisher由硬件通信的launch文件启动。
'''

def generate_launch_description():
    # ========= MoveIt Config =========
    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="moveit_robot_control")
        .robot_description(file_path="urdf/fr3.urdf.xacro")
        .robot_description_semantic(file_path="moveit_config/fr3.srdf.xacro")
        .robot_description_kinematics(file_path="moveit_config/kinematics.yaml")
        .joint_limits(file_path="moveit_config/fr3_joint_limits.yaml")
        .trajectory_execution(file_path="moveit_config/fr3_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ========= Servo Container Node =========
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit_robot_control")
        .yaml("moveit_config/fr3_servo_config.yaml")
        .to_dict()
    }

    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "fr3_arm"}

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    acceleration_filter_update_period,
                    planning_group_name,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            ),
        ],
        output="screen",
    )

    # ========= Launch Description =========
    ld = LaunchDescription()

    ld.add_action(container)

    return ld