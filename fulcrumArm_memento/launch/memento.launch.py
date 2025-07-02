from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from srdfdom.srdf import SRDF
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

def load_file(package_name, file_path):
    package_name = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_name, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None
    
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError: 
        return None
    
class FulcrumArmMoveitConfig:
    def __init__(self, moveit_config, ros2_controllers):
        self.moveit_config = moveit_config
        self.ros2_controllers = ros2_controllers

def get_moveit_config(robot_name: str):
    base_path = "config"

    moveit_config = {
        MoveItConfigsBuilder(robot_name, package_name="fulcrumArm_moveit")
        .robot_description(file_path=f"{base_path}/{robot_name}.urdf.xacro")
        .robot_description_semantic(file_path=f"{base_path}/{robot_name}.srdf")
        .robot_description_kinematics(file_path=f"{base_path}/kinematics.yaml")
        .trajectory_execution(file_path=f"{base_path}/moveit_controllers.yaml")
        .sensors_3d(file_path=f"{base_path}/sensors_3d.yaml")
        .joint_limits(file_path=f"{base_path}/joint_limits.yaml")
        .pilz_cartesian_limits(file_path=f"{base_path}/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    }

    share_dir = get_package_share_directory("fulcrumArm_moveit")
    ros2_controllers = os.path.join(share_dir, f"{base_path}/ros2_controllers.yaml")
    moveit_config = FulcrumArmMoveitConfig(moveit_config, ros2_controllers)
    
    return moveit_config

def generate_launch_description():

    FULCRUMARM_MODEL = os.environ['FULCRUMARM_MODEL']
    moveit_config = get_moveit_config(FULCRUMARM_MODEL)
    rviz_config = os.path.join(get_package_share_directory('fulcrumArm_memento'), 'rviz', 'memento.rviz')

    fulcrumArm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fulcrumArm_moveit', 'launch', 'fulcrumArm_moveit.launch.py'))
        ),
        launch_arguments={
            'rviz_config': rviz_config,
        }.items()
    )    

    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.moveit_config.robot_description],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            fulcrumArm_moveit_launch,
            run_move_group_node,
        ]
    )