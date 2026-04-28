from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rm_75_dual", package_name="rm75_dual_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
