from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    arduinobot_description_dir = get_package_share_directory('arduinobot_description')
    # Declare a launch argument for the URDF model path
    # This allows specifying a different model file when launching
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(arduinobot_description_dir, 'urdf', 'arduinobot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    # Create a parameter containing the URDF model content, if you have .urdf file
    # robot_description = ParameterValue(Command(['cat', LaunchConfiguration('model')]))
    # Uses xacro to process the model file
    robot_description = ParameterValue(Command(['xacro', ' ', LaunchConfiguration('model')]))

    # Launch the robot state publisher node
    # This node publishes the robot state to tf2
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # Launch the joint state publisher GUI
    # This provides a GUI to manually set joint positions
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Launch RViz with a custom configuration
    # This provides 3D visualization of the robot
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('arduinobot_description'), 'rviz', 'display.rviz')]
    )
    
    # Return a launch description containing all the nodes
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])