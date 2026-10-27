import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    panda_moveit_share = get_package_share_directory('moveit_resources_panda_moveit_config')

    # Arguments
    auto_orchestrate_arg = DeclareLaunchArgument(
        'auto_orchestrate',
        default_value='true',
        description='Whether to automatically run the bin-picking orchestrator cycle'
    )

    # Include Official MoveIt 2 Panda Demo (MoveGroup + Fake Hardware + Controllers + RViz)
    panda_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(panda_moveit_share, 'launch', 'demo.launch.py')
        )
    )

    # 1. Workcell Planning Scene Node (Table, Bin Walls, Pallet)
    planning_scene_node = Node(
        package='vision_guided_bin_picking',
        executable='planning_scene_node',
        name='planning_scene_node',
        output='screen'
    )

    # 2. 3D Vision Detector Node (Object & Grasp Pose Estimation)
    vision_detector_node = Node(
        package='vision_guided_bin_picking',
        executable='vision_detector_node',
        name='vision_detector_node',
        output='screen'
    )

    # 3. Bin-Picking Task Orchestrator Node (FSM Coordinator)
    orchestrator_node = Node(
        package='vision_guided_bin_picking',
        executable='bin_picking_orchestrator_node',
        name='bin_picking_orchestrator_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_orchestrate'))
    )

    return LaunchDescription([
        auto_orchestrate_arg,
        panda_demo_launch,
        planning_scene_node,
        vision_detector_node,
        orchestrator_node,
    ])
