"""Launch runtime components from the selected model's actual configuration."""
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from .model_profile import load_model_profile, runtime_parameters


def runtime_nodes(context, navigation=False):
    root = LaunchConfiguration('project_root').perform(context)
    model_id = LaunchConfiguration('model_id').perform(context)
    cfg = load_model_profile(root, model_id)
    bev, safety, local = runtime_parameters(cfg, navigation)
    bev.update(goal_frame='map' if navigation else LaunchConfiguration('goal_frame').perform(context),
               goal_x=float(LaunchConfiguration('initial_x' if navigation else 'goal_x').perform(context)),
               goal_y=float(LaunchConfiguration('initial_y' if navigation else 'goal_y').perform(context)))
    nodes = [Node(package='bev_pkg', executable='bev_node', name='bev_node', output='screen', parameters=[bev]),
             Node(package='safety_pkg', executable='safety_node', name='safety_node', output='screen', parameters=[safety])]
    if navigation:
        nodes.extend([
            Node(package='rl_nav_pkg', executable='route_manager', name='route_manager', output='screen', parameters=[{'use_sim_time': True}]),
            Node(package='rl_nav_pkg', executable='local_goal_node', name='local_goal_node', output='screen', parameters=[local])])
    command = ['bash', root + '/training/scripts/run_policy.sh', model_id,
               '-p', 'goal_mode:=' + ('local' if navigation else 'final'),
               '-p', 'require_navigation_status:=' + ('true' if navigation else 'false'),
               '-p', 'goal_stop_radius:=' + str(cfg['environment']['goal_radius']),
               '-p', 'goal_hold_seconds:=' + str(cfg['environment']['goal_hold_seconds'])]
    nodes.append(ExecuteProcess(condition=IfCondition(LaunchConfiguration('policy')), cmd=command, output='screen'))
    return nodes
