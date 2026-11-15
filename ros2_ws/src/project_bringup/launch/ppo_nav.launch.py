import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rl_nav_pkg.launch_support import runtime_nodes


def generate_launch_description():
    project_root = LaunchConfiguration('project_root')
    model_id = LaunchConfiguration('model_id')
    use_rviz = LaunchConfiguration('rviz')
    run_policy = LaunchConfiguration('policy')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

    wpr_share = get_package_share_directory('wpr_simulation2')
    map_yaml = os.path.join(wpr_share, 'maps', 'map.yaml')
    nav2_params = os.path.join(wpr_share, 'config', 'nav2_params.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wpr_share, 'launch', 'robocup_home.launch.py')))

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {'yaml_filename': map_yaml, 'use_sim_time': True}])

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params,
            {
                'use_sim_time': True,
                'set_initial_pose': True,
                'initial_pose.x': ParameterValue(initial_x, value_type=float),
                'initial_pose.y': ParameterValue(initial_y, value_type=float),
                'initial_pose.z': 0.0,
                'initial_pose.yaw': ParameterValue(initial_yaw, value_type=float),
            },
        ])

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': True}])

    localization_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }])

    navigation_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['planner_server'],
        }])

    runtime = OpaqueFunction(function=lambda context: runtime_nodes(context, navigation=True))

    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(wpr_share, 'rviz', 'navi.rviz')],
                condition=IfCondition(use_rviz),
                output='screen')])

    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=os.path.expanduser('~/ROS2hw')),
        DeclareLaunchArgument('model_id', default_value='ppo_v2_recommended'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('policy', default_value='true'),
        # Spawn is at (-6.0, -0.5), exactly on the map edge; AMCL starts 0.1 m inside
# so the global planner does not reject the start pose as out of costmap.
        DeclareLaunchArgument('initial_x', default_value='-5.9'),
        DeclareLaunchArgument('initial_y', default_value='-0.5'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        simulation,
        map_server,
        amcl,
        localization_manager,
        planner,
        navigation_manager,
        runtime,
        rviz,
    ])
