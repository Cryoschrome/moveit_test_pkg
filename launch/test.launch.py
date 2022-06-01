import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(package_name="moveit_test_pkg", robot_name="single_joint")
        .robot_description(file_path="config/single_joint.urdf")
        .trajectory_execution(file_path="config/fake_controllers.yaml")
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_test_pkg",
        executable="move_group_test",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "world", "panda_link0"],
    # )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    return LaunchDescription([
        #static_tf,
        robot_state_publisher,
        run_move_group_node
    ])