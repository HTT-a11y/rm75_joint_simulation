import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 找到官方 Gazebo 启动文件的真实路径
    gazebo_launch_path = os.path.join(
        get_package_share_directory('rm_75_config'),
        'launch',
        'gazebo_moveit_demo.launch.py'
    )

    return LaunchDescription([
        # 魔法指令：全局广播，强行将 MTC 执行器插件注入到所有叫 move_group 的节点中
        SetParameter(name='capabilities', value='moveit_task_constructor/ExecuteTaskSolutionCapability'),
        
        # 拉起官方的原生环境
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        )
    ])
