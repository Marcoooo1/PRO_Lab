"""
Startet Undock + alle 3 Filter + Logger + Auto-Driver.

Aufruf:
  ros2 launch prob_robotics_bringup all_filters.launch.py \
        config_file:=filters_wrong_init.yaml \
        pattern:=circle \
        duration:=30.0 \
        csv_path:=/tmp/log_wrong_init_A.csv

WICHTIG: Der TurtleBot4 ignoriert externe cmd_vel-Befehle solange er
"gedockt" ist. Der Undock-Vorgang beinhaltet eine Drehbewegung; der
auto_driver darf erst NACH deren Abschluss starten.

FIX: 'ros2 action send_goal' kehrt mit --feedback zurueck, sobald das
Ergebnis (nicht nur die Annahme) des Goals vorliegt -- das blockiert
zuverlaessig bis der Undock-Vorgang wirklich fertig ist. Der auto_driver
startet danach per OnProcessExit-Event.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('prob_robotics_bringup')

    config_file = LaunchConfiguration('config_file')
    pattern     = LaunchConfiguration('pattern')
    duration    = LaunchConfiguration('duration')
    csv_path    = LaunchConfiguration('csv_path')
    v           = LaunchConfiguration('v')
    w           = LaunchConfiguration('w')

    config_path = PathJoinSubstitution([pkg_share, 'config', config_file])

    # --feedback erzwingt Warten auf das tatsaechliche Ergebnis des Goals,
    # nicht nur auf die Annahme. So blockiert der Prozess bis Undock fertig ist.
    undock_action = ExecuteProcess(
        cmd=['ros2', 'action', 'send_goal', '--feedback',
             '/undock', 'irobot_create_msgs/action/Undock', '{}'],
        output='screen',
    )

    filter_nodes = [
        Node(
            package='prob_robotics_filters',
            executable='kf_node',
            name='kf_node',
            parameters=[config_path],
            output='screen',
        ),
        Node(
            package='prob_robotics_filters',
            executable='ekf_node',
            name='ekf_node',
            parameters=[config_path],
            output='screen',
        ),
        Node(
            package='prob_robotics_filters',
            executable='pf_node',
            name='pf_node',
            parameters=[config_path],
            output='screen',
        ),
        Node(
            package='prob_robotics_eval',
            executable='trajectory_logger',
            name='trajectory_logger',
            parameters=[{'output_csv': csv_path, 'rate_hz': 20.0}],
            output='screen',
        ),
    ]

    auto_driver_node = Node(
        package='prob_robotics_eval',
        executable='auto_driver',
        name='auto_driver',
        parameters=[{
            'pattern': pattern,
            'duration': duration,
            'v': v,
            'w': w,
        }],
        output='screen',
    )

    start_driver_after_undock = RegisterEventHandler(
        OnProcessExit(
            target_action=undock_action,
            on_exit=[auto_driver_node],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='filters_wrong_init.yaml'),
        DeclareLaunchArgument('pattern',     default_value='circle'),
        DeclareLaunchArgument('duration',    default_value='30.0'),
        DeclareLaunchArgument('csv_path',    default_value='/tmp/trajectory_log.csv'),
        DeclareLaunchArgument('v',           default_value='0.2'),
        DeclareLaunchArgument('w',           default_value='0.3'),

        undock_action,
        *filter_nodes,
        start_driver_after_undock,
    ])
