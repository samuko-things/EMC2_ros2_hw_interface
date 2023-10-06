import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue





def generate_launch_description():
    # delare any path variable
    my_pkg_path = get_package_share_directory('diff_bot')
    
    rviz_config_file = os.path.join(my_pkg_path,'config','robot_rviz_sim_view.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

     # Create the launch description and populate
    ld = LaunchDescription()
    

    # Add the nodes to the launch description
    ld.add_action(rviz_node)
    
    return ld      # return (i.e send) the launch description for excecution

