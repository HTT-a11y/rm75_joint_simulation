from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 终极大杀器：使用官方自带的 Builder 自动搜刮并解析 rm_75_config 里的所有配置
    # (这个名字是我刚刚去你的官方 demo.launch.py 里查证过的，绝对正确)
    moveit_config = MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config").to_moveit_configs()

    mtc_node = Node(
        package='motion_planning',
        executable='planner_node',
        output='screen',
        # to_dict() 会自动把 URDF、SRDF、运动学、OMPL 等几百个参数完美注入节点！
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([mtc_node])
