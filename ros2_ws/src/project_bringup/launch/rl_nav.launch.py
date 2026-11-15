import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from rl_nav_pkg.launch_support import runtime_nodes


def generate_launch_description():
    project_root = LaunchConfiguration('project_root')
    model_id = LaunchConfiguration('model_id')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_frame = LaunchConfiguration('goal_frame')
    use_rviz = LaunchConfiguration('rviz')
    run_policy = LaunchConfiguration('policy')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('wpr_simulation2'),
                'launch',
                'wpb_simple.launch.py')))

    runtime = OpaqueFunction(function=lambda context: runtime_nodes(context, navigation=False))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(get_package_share_directory('project_bringup'), 'rviz', 'rl_nav.rviz')],
        condition=IfCondition(use_rviz),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=os.path.expanduser('~/ROS2hw')),
        DeclareLaunchArgument('model_id', default_value='split_mixed_pretrain_v1'),
        DeclareLaunchArgument('goal_x', default_value='0.0'),
        DeclareLaunchArgument('goal_y', default_value='2.0'),
        DeclareLaunchArgument('goal_frame', default_value='odom'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('policy', default_value='true'),
        simulation,
        runtime,
        rviz,
    ])
