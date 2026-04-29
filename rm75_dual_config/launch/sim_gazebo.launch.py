import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # ======================================================================
    # 💥 让官方 Builder 来处理 YAML，确保命名空间和层级绝对正确！
    # ======================================================================
    moveit_config = (
        MoveItConfigsBuilder("rm_75_dual", package_name="rm75_dual_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml") # 这里读回那份包含控制器的文件
        .to_moveit_configs()
    )
    
    # 转换为字典并追加 MTC 执行能力
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["capabilities"] = "move_group/ExecuteTaskSolutionCapability"
    moveit_config_dict["use_sim_time"] = True

    # 注入 Gazebo 模型路径，防止找不到 Mesh
    install_dir = os.path.abspath(os.path.join(get_package_share_directory('rm_description'), '..', '..', '..'))
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', install_dir, '/share']
    )

    # 1. 启动 Gazebo (静默模式，防止声音报错卡死)
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'verbose': 'true', 'pause': 'false'}.items()
    )

    # 2. 状态发布
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config_dict],
    )

    # 3. 生成物理实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rm_75_dual', '-topic', 'robot_description'],
        output='screen'
    )

    # ========================================================
    # 4. 延迟启动：确保参数完全载入 MoveGroup
    # ========================================================
    delayed_nodes = TimerAction(
        period=12.0,
        actions=[
            Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
            Node(package="controller_manager", executable="spawner", arguments=["left_arm_controller"]),
            Node(package="controller_manager", executable="spawner", arguments=["right_arm_controller"]),
            Node(
                package="moveit_ros_move_group", 
                executable="move_group", 
                output="screen", 
                # 这里必须是完整的 moveit_config_dict
                parameters=[moveit_config_dict] 
            ),
            Node(
                package="rviz2", 
                executable="rviz2", 
                name="rviz2", 
                output="screen", 
                parameters=[moveit_config_dict],
                arguments=["-d", os.path.join(get_package_share_directory("rm75_dual_config"), "config", "moveit.rviz")],
            )
        ]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        rsp_node,
        spawn_entity,
        delayed_nodes
    ])
