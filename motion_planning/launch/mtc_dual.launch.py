from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 提取双臂模型配置
    moveit_config = MoveItConfigsBuilder("rm_75_dual", package_name="rm75_dual_config").to_moveit_configs()

    # 启动 MTC 节点，并喂入极其重要的图纸参数！
    mtc_node = Node(
        package="motion_planning",
        executable="planner_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([mtc_node])
