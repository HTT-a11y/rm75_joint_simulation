import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 获取仿真时间参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ======================================================================
    # 💥 核心修复：使用 Builder 提取完整的 OMPL 和逆运动学参数，并喂给 MTC 节点
    # ======================================================================
    moveit_config = (
        MoveItConfigsBuilder("rm_75_dual", package_name="rm75_dual_config")
        .planning_pipelines(pipelines=["ompl"]) # 强制指定加载 OMPL 规划管道
        .to_moveit_configs()
    )
    
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["use_sim_time"] = use_sim_time

    # 定义 MTC 规划节点
    planner_node = Node(
        package='motion_planning',
        executable='planner_node',       # 你的 C++ 可执行文件名
        name='mtc_dual_arm_planner',
        output='screen',
        parameters=[moveit_config_dict]  # 💥 极其关键：把包含 OMPL 的参数字典喂进去
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        planner_node
    ])
