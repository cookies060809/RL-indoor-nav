import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        parameters=[nav2_params])

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
            'goal_frame': 'map',
            'goal_x': ParameterValue(initial_x, value_type=float),
            'goal_y': ParameterValue(initial_y, value_type=float),
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

    route_manager = Node(
        package='rl_nav_pkg',
        executable='route_manager',
        name='route_manager',
        output='screen',
        parameters=[{'use_sim_time': True}])

    local_goal = Node(
        package='rl_nav_pkg',
        executable='local_goal_node',
        name='local_goal_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'lookahead': 1.2}])

    policy = ExecuteProcess(
        condition=IfCondition(run_policy),
        cmd=[
            'bash',
            PathJoinSubstitution([project_root, 'training', 'scripts', 'run_policy.sh']),
            model_id,
        ],
        output='screen')

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
        DeclareLaunchArgument('model_id', default_value='split_mixed_pretrain_v1'),
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
        bev,
        safety,
        route_manager,
        local_goal,
        policy,
        rviz,
    ])