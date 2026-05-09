from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Gestionnaire de mode — écoute /rover/mode depuis le PC
        Node(
            package='rover_xplore',
            executable='mode_manager_node',
            name='mode_manager_node',
            output='screen',
        ),

        # Contrôle des roues — actif en race / arm / autonomous
        Node(
            package='rover_xplore',
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen',
        ),

        # Bras robotique — actif en arm / autonomous
        Node(
            package='rover_xplore',
            executable='arm_node',
            name='arm_node',
            output='screen',
        ),

        # Pont série RPi ↔ Arduino — toujours actif (owner du port série)
        Node(
            package='rover_xplore',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
        ),

        # Détection ArUco — actif en autonomous uniquement
        Node(
            package='rover_xplore',
            executable='aruco_node',
            name='aruco_node',
            output='screen',
        ),

        # Caméra — actif en race / arm / autonomous
        # Note : à lancer nativement hors Docker si picamera2 est utilisé
        Node(
            package='rover_xplore',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),

    ])
