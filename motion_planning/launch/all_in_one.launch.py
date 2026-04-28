import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 启动物理世界 (直接拉起 Gazebo，不再经过多余的封装)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rm_gazebo'), 'launch', 'gazebo_75_demo.launch.py')])
    )

    # 2. 提取官方的基础参数
    moveit_config = MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config").to_moveit_configs()

    # 3. 显式创建 MoveGroup 节点！(打破黑盒，强行喂入 MTC 插件)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"}, # <--- 绝对生效的终极魔法
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )

    # 4. 显式拉起 RViz 界面
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory("rm_75_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        gazebo_launch,
        move_group_node,
        rviz_node
    ])
