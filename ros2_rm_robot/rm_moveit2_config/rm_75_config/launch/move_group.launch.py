import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 自动搜刮所有配置
    moveit_config = (
        MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config")
        .robot_description(file_path="config/rm_75_description.urdf.xacro")
        .robot_description_semantic(file_path="config/rm_75_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 2. 将配置转为字典
    moveit_config_dict = moveit_config.to_dict()

    # ========================================================
    # 💥 最关键的一步：暴力覆盖字典里的 capabilities 参数！💥
    # 不管官方怎么藏，我们直接在最后一刻把 MTC 插件写进字典里
    # ========================================================
    moveit_config_dict["capabilities"] = "moveit_task_constructor/ExecuteTaskSolutionCapability"
    
    # 强制开启仿真时间同步
    moveit_config_dict["use_sim_time"] = True

    # 3. 显式创建 move_group 节点，并注入我们修改过的心脏
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
    )

    return LaunchDescription([move_group_node])
