import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):

    # Configuration
    device = LaunchConfiguration("device")
    rate = LaunchConfiguration("rate")
    rosbag_path = LaunchConfiguration("rosbag_path")

    # Node from package audio common
    audio_node = Node(
        package='audio_common',
        executable='audio_player_node',
        parameters=[{
            "device": device,
            "rate": rate,
        }],
        output='screen'
    )

    # Rosbag play command
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            rosbag_path
        ],
        output='screen'
    )

    layout_path = PathJoinSubstitution([
        FindPackageShare('ut_start_2'),  
        'config',
        'visualization.xml'
    ])

    # PlotJuggler with a layout file
    plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        arguments=[
            '--nosplash',
            '--layout',
            layout_path,
        ],
        output='screen'
    )

    to_start = [
        audio_node,
        rosbag_play,
        plotjuggler
    ]

    return to_start


def generate_launch_description():

    # Arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "device",
            default_value="-1",
            description="Device used to acquire audio"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rate",
            default_value="16000",
            description="Frequency of the audio to be acquired"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rosbag_path",
            default_value=os.path.expanduser("~") + "/latest",
            description="Full path of the rosbag"
        )
    )    

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
