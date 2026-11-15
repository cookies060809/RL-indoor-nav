import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


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

    bev = Node(
        package='bev_pkg',
        executable='bev_node',
        name='bev_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'n': 32,
            'extent': 4.8,
            'spec_version': 'bev_nav_v1',
            'beams': 360,
            'robot_radius': 0.226,
            'distance_clip': 1.0,
            'goal_distance_scale': 5.0,
            'v_max': 0.26,
            'w_max': 1.0,
            'goal_frame': goal_frame,
            'goal_x': goal_x,
            'goal_y': goal_y,
        }])

    safety = Node(
        package='safety_pkg',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'candidate_topic': '/cmd_vel_rl',
            'v_max': 0.26,
            'w_max': 1.0,
            'acceleration': 0.5,
            'angular_acceleration': 2.0,
            'control_dt': 0.05,
            'stale_timeout': 0.5,
            'sensor_timeout': 0.5,
            'robot_radius': 0.226,
            'safety_margin': 0.04,
            'reaction_time': 0.2,
            'brake_deceleration': 0.8,
        }])

    policy = ExecuteProcess(
        condition=IfCondition(run_policy),
        cmd=[
            'bash',
            PathJoinSubstitution([project_root, 'training', 'scripts', 'run_policy.sh']),
            model_id,
        ],
        output='screen')

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
        bev,
        safety,
        policy,
        rviz,
    ])
