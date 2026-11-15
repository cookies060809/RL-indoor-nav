import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NAV2_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def generate_launch_description():
    rviz_enabled = LaunchConfiguration('rviz')
    safety_enabled = LaunchConfiguration('safety')
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

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')])

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params])

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params])

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')])

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params])

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params])

    # Plan A: controller -> /cmd_vel_nav -> smoother -> /cmd_vel_nav_smoothed -> safety -> /cmd_vel
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel_nav_smoothed'),
        ])

    navigation_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': NAV2_NODES,
        }])

    safety = Node(
        package='safety_pkg',
        executable='safety_node',
        name='safety_node',
        output='screen',
        condition=IfCondition(safety_enabled),
        parameters=[{
            'use_sim_time': True,
            'candidate_topic': '/cmd_vel_nav_smoothed',
            'strict_scan_coverage': True,
            'canonical_beams': 360,
            'range_min': 0.24,
            'range_max': 10.0,
            'v_max': 0.26,
            'w_max': 1.0,
            'acceleration': 0.8,
            'angular_acceleration': 2.5,
            'control_dt': 0.05,
            'stale_timeout': 0.5,
            'sensor_timeout': 0.5,
            'robot_radius': 0.226,
            'safety_margin': 0.04,
            'reaction_time': 0.2,
            'brake_deceleration': 0.8,
        }])

    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(wpr_share, 'rviz', 'navi.rviz')],
                condition=IfCondition(rviz_enabled),
                output='screen')])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('safety', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='-6.0'),
        DeclareLaunchArgument('initial_y', default_value='-0.5'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        simulation,
        map_server,
        amcl,
        localization_manager,
        controller,
        smoother,
        planner,
        behavior,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_manager,
        safety,
        rviz,
    ])
