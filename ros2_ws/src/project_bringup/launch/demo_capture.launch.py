import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output = LaunchConfiguration('output')
    topic = LaunchConfiguration('topic')
    fps = LaunchConfiguration('fps')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')

    share = get_package_share_directory('project_bringup')
    model = os.path.join(share, 'models', 'overhead_camera', 'model.sdf')
    script = os.path.join(share, 'scripts', 'record_video.py')

    spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_demo_camera',
        output='screen',
        arguments=[
            '-file', model,
            '-entity', 'demo_camera',
            '-x', camera_x,
            '-y', camera_y,
            '-z', camera_z,
        ])

    recorder = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    '/usr/bin/python3',
                    script,
                    '--topic', topic,
                    '--output', output,
                    '--fps', fps,
                ],
                output='screen')])

    return LaunchDescription([
        DeclareLaunchArgument(
            'output',
            default_value=os.path.expanduser('~/ROS2hw/experiments/videos/demo.mp4')),
        DeclareLaunchArgument('topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('fps', default_value='5.0'),
        DeclareLaunchArgument('camera_x', default_value='0.0'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='6.5'),
        spawn_camera,
        recorder,
    ])