from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bot_hardware',
            namespace='DonkeyJet',
            executable='joy',
            name='joystick'
        ),
        Node(
            package='bot_hardware',
            namespace='DonkeyJet',
            executable='pca9685',
            name='actuation'
        )
    ])