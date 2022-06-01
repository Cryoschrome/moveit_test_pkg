import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
  robot_description_config = load_file(
    "moveit_test_pkg", "urdf/one_joint.urdf"
  )
  robot_description = {"robot_description": robot_description_config}

  robot_description_semantic_config = load_file(
    "moveit_test_pkg", "config/single_joint.srdf"
  )
  robot_description_semantic = {
    "robot_description_semantic": robot_description_semantic_config
  }

  kinematics_yaml = load_yaml(
    "moveit_test_pkg", "config/kinematics.yaml"
  )
  robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

  #OPML
  ompl_planning_pipeline_config = {
    "move_group": {
      "planning_plugin": "ompl_interface/OMPLPlanner",
      "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
      "start_state_max_bounds_error": 0.1,
    }
  }
  ompl_planning_yaml = load_yaml(
    "moveit_test_pkg", "config/ompl_planning.yaml"
  )
  ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

  #Trajectory execution
  moveit_simple_controllers_yaml = load_yaml(
    "moveit_test_pkg", "config/ros_controllers.yaml"
  )
  moveit_controllers = {
    "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
    "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
  }

  trajectory_execution = {
    "moveit_manage_controllers": True,
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    "trajectory_execution.allowed_goal_duration_margin": 0.5,
    "trajectory_execution.allowed_start_tolerance": 0.01,
  }

  planning_scene_monitor_parameters = {
    "publish_planning_scene": True,
    "publish_geometry_updates": True,
    "publish_state_updates": True,
    "publish_transforms_updates": True,
  }

  move_group_node = Node(package='moveit_test_pkg', executable='move_group_test',
    output='screen',
    parameters=[
      robot_description,
      robot_description_semantic,
      kinematics_yaml,
      ompl_planning_pipeline_config,
      trajectory_execution,
      moveit_controllers,
      planning_scene_monitor_parameters,
    ]
  )

  robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
          robot_description,
        ]
    )

  return LaunchDescription([
        robot_state_publisher,
        move_group_node
    ])