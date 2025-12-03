import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, 
    DeclareLaunchArgument, 
    RegisterEventHandler, 
    OpaqueFunction
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def create_symlink(context, bag_path):

    link_path = os.path.dirname(bag_path) + "/latest"

    if os.path.islink(link_path) or os.path.exists(link_path):
        os.unlink(link_path)

    os.symlink(bag_path, link_path)

    print(f"[INFO] Created symlink {link_path} -> {bag_path}")

    return []


def launch_setup(context):

    # Configuration
    device = LaunchConfiguration("device")
    rate = LaunchConfiguration("rate")
    rosbag_dir = LaunchConfiguration("rosbag_dir")

    # Node from package audio common
    audio_node = Node(
        package='audio_common',
        executable='audio_capturer_node',
        parameters=[{
            "device": device,
            "rate": rate,
        }],
        output='screen'
    )

    # Rosbag record command
    rosbag_path = rosbag_dir.perform(context) + "/rosbag2_" + datetime.today().strftime('%Y_%m_%d_%H_%M_%S')
   
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/audio', 
            '/fmu/out/sensor_combined',
            '/fmu/out/vehicle_global_position',
            '/fmu/out/vehicle_odometry',
            '--max-bag-duration', '300',
            '--output', rosbag_path,
        ],
        output='screen'
    )

    # Create the symbolink link that points to the latest rosbag
    symlink_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=rosbag_record,
            on_exit=[OpaqueFunction(function=create_symlink, kwargs={"bag_path": rosbag_path})]
        )
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
        rosbag_record,
        symlink_handler,
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
            "rosbag_dir",
            default_value=os.path.expanduser("~"),
            description="Directory where to save the rosbag"
        )
    )    

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
