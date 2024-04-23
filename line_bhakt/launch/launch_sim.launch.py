import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name='line_bhakt' #<--- CHANGE ME

    descpkg = 'autocar_description'
    wrldpkg = 'autocar_gazebo'

    world = os.path.join(get_package_share_directory(wrldpkg),'worlds', 'clover_field.world')
    urdf = os.path.join(get_package_share_directory(descpkg),'urdf', 'actor.urdf')
    rviz = os.path.join(get_package_share_directory(descpkg), 'rviz', 'empty.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    # Launch them all!
    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),

        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),

        ExecuteProcess(
            cmd=['gazebo', world, 'libgazebo_ros_factory.so', 'libgazebo_ros_init.so'],
        ),
        
        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([os.path.join(
        #             get_package_share_directory('gazebo_ros'),'launch', 'gazebo.launch.py')]),
        #         launch_arguments={'gui': 'false'}.items()
        #      ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        ),
        
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     name='spawn',
        #     arguments=['-topic', 'robot_description','-entity', 'navigation_bot'],
        #     output = 'screen'
        # ),
        
        Node(
            package=package_name,
            executable='ImageCapture',
            name='Lane'
        ),
        
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        # ),
    
    ])
    
def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
    
