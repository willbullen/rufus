from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',
            description='Run in simulation mode without hardware'
        ),
        
        Node(
            package='rufus_drivers',
            executable='ddsm315_driver',
            name='ddsm315_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'track_width': 0.48,
                'wheel_radius': 0.05,
                'simulation_mode': LaunchConfiguration('simulation_mode')
            }]
        ),
        
        Node(
            package='rufus_drivers',
            executable='st3215_driver',
            name='st3215_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baudrate': 1000000,
                'simulation_mode': LaunchConfiguration('simulation_mode')
            }]
        ),
    ])

