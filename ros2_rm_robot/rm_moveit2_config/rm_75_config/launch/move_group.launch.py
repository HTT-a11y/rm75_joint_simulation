import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 读取官方所有默认配置
    moveit_config = MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config").to_moveit_configs()

    # 暴力声明 move_group 节点，强行塞入 MTC 插件！
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # 这就是缺失的“耳朵”：开启 MTC 动作服务器
            {"capabilities": "moveit_task_constructor/ExecuteTaskSolutionCapability"},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group_node])
