import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the rosbridge_websocket_launch.xml
    rosbridge_launch_path = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    )

    # Include the rosbridge_websocket_launch.xml
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_path)
    )

    # Define the driver_node
    driver_node = Node(
        package='mr2_drive',
        executable='driver_node',
        name='driver_node',
        output='screen'
    )

    # Define the steering_node
    steering_node = Node(
        package='mr2_drive',
        executable='steering_node',
        name='steering_node',
        output='screen'
    )

    # Define the mr2_drive_motor_interface node
    drive_motor_interface = Node(
        package='mr2_drive_motor',
        executable='mr2_drive_motor_interface',
        name='mr2_drive_motor_interface',
        output='screen'
    )

    # Define the dynamixel_controller node
    dynamixel_controller = Node(
        package='mr2_dynamixel',
        executable='dynamixel_controller',
        name='dynamixel_controller',
        output='screen'
    )

    # Define the usb_cam_node_exe with the pixel_format parameter
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node_exe',
        output='screen',
        parameters=[{'pixel_format': 'yuyv'}]
    )

    # Define the web_video_server node
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    # Create and populate the launch description
    ld = LaunchDescription()

    # Add the included launch file
    ld.add_action(rosbridge_launch)

    # Add all other nodes
    ld.add_action(driver_node)
    ld.add_action(steering_node)
    ld.add_action(dynamixel_controller)
    # ld.add_action(drive_motor_interface)
    ld.add_action(usb_cam_node)
    ld.add_action(web_video_server_node)

    return ld
