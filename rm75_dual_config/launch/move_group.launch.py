import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 使用 Builder 提取配置
    moveit_config = MoveItConfigsBuilder("rm_75_dual", package_name="rm75_dual_config").to_moveit_configs()
    moveit_config_dict = moveit_config.to_dict()

    # ==========================================
    # 强制覆盖参数
    # ==========================================
    moveit_config_dict["capabilities"] = "move_group/ExecuteTaskSolutionCapability"
    moveit_config_dict["use_sim_time"] = False

    # 启动 MoveGroup 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
    )

    # 启动 Robot State Publisher (解析并发布所有的 tf 骨架)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config_dict],
    )

    # 💥 找到 RViz 的配置文件路径 💥
    rviz_config_file = os.path.join(
        get_package_share_directory("rm75_dual_config"),
        "config",
        "moveit.rviz",
    )

    # 启动 RViz 节点 (带上地图！)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[moveit_config_dict],
        arguments=["-d", rviz_config_file], # 💥 就在这里！
    )

    # 启动一个简易的 Joint State Publisher
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[moveit_config_dict],
    )

    # 强制发布 world -> table_link 的相对关系
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "table_link"],
    )

    return LaunchDescription([
        move_group_node,
        rsp_node,
        rviz_node,
        jsp_node,
        static_tf_node
    ])
