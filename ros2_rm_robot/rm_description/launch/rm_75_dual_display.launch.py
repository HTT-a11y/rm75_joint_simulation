import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 极其严谨的模型解析命令 (确保 xacro 正确执行并转为纯 URDF 字符串)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rm_description"), "urdf", "rm_75_dual.urdf.xacro"]
            ),
        ]
    )
    
    # 2. 核心：必须明确声明这个字典
    robot_description = {"robot_description": robot_description_content}

    # 3. 状态发布节点 (强行开启 use_sim_time 确保时间戳不出错)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}], # 显示真实模型不需要仿真时间
    )

    # 4. GUI 滑动条节点 (极其关键：它也必须读取模型，才知道有哪些滑块要生成！)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[robot_description], # 如果没有这行，GUI 可能连滑块都生成不出来！
    )

    # 5. RViz 可视化节点
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rm_description"), "urdf", "display_arm.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
