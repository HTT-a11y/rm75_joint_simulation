import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    # 这些是 MTC 必须吃下去的配置参数
    robot_description_semantic_config = load_yaml('rm_moveit2_config', 'rm_75_config/config/rm_75_description.srdf')
    kinematics_yaml = load_yaml('rm_moveit2_config', 'rm_75_config/config/kinematics.yaml')
    joint_limits_yaml = load_yaml('rm_moveit2_config', 'rm_75_config/config/joint_limits.yaml')
    
    # OMPL 规划流水线配置
    ompl_planning_pipeline_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }

    # 启动你的 MTC 节点，并注入灵魂（参数）
    mtc_node = Node(
        package='motion_planning',
        executable='planner_node',
        output='screen',
        parameters=[
            {'robot_description_semantic': robot_description_semantic_config},
            {'robot_description_kinematics': kinematics_yaml},
            {'robot_description_planning': joint_limits_yaml},
            ompl_planning_pipeline_config,
        ],
    )

    return LaunchDescription([mtc_node])
