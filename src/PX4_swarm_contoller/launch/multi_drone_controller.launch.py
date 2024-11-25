import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the path to the PX4-Autopilot directory
    px4_path = os.path.expanduser("~/PX4-Autopilot")

    # Path to the script
    sitl_script = os.path.join(px4_path, "Tools", "simulation", "gazebo-classic", "sitl_multiple_run.sh")
    
    # Declare arguments
    arguments = [
        DeclareLaunchArgument('nb_vehicles', default_value='5', description='Number of vehicles'),
        DeclareLaunchArgument('drone_model', default_value='iris', description='Drone model'),
        DeclareLaunchArgument('world', default_value='', description='World file name'),
        DeclareLaunchArgument('target', default_value='', description='Target for the mission'),
        DeclareLaunchArgument('label', default_value='', description='Mission label'),
    ]
    
    # Command to execute the script with arguments
    execute_sitl_script = ExecuteProcess(
        cmd=[
            "/bin/bash", sitl_script,
            '-n', LaunchConfiguration('nb_vehicles'),
            '-m', LaunchConfiguration('drone_model'),
            '-w', LaunchConfiguration('world'),
            '-t', LaunchConfiguration('target'),
            '-l', LaunchConfiguration('label'),
        ],
        cwd=px4_path,  # Set working directory to PX4-Autopilot
        output="screen"
    )

    # Launch MicroXRCEAgent in a new terminal
    micro_ros_agent = ExecuteProcess(
        cmd=[
            "gnome-terminal",  # Use your preferred terminal emulator (e.g., xterm, konsole)
            "--",  # Separator for terminal arguments and command
            "MicroXRCEAgent", "udp4", "-p", "8888"
        ],
        output="screen"
    )

    return LaunchDescription(arguments + [execute_sitl_script, micro_ros_agent])
