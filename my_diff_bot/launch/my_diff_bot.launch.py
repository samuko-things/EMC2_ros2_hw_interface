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
    my_pkg_path = get_package_share_directory('my_diff_bot')
    
    rviz_config_file = os.path.join(my_pkg_path,'config','robot_rviz_sim_view.rviz')

  
  
    use_sim_time = 'false'
    use_ros2_control = 'true'
    # create needed nodes or launch files
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(my_pkg_path,'launch','rsp.launch.py')]
            ), 
            launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )



    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("my_diff_bot"), "urdf", "urdf_description.xacro"]
    #         ),
    #     ]
    # )

    robot_description_content = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    robot_description_xml = ParameterValue(robot_description_content, value_type=str)

    robot_description = {"robot_description": robot_description_xml}

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("my_diff_bot"),
    #         "config",
    #         "diff_drive_controller.yaml",
    #     ]
    # )

    robot_controllers = os.path.join(my_pkg_path,'config','diff_drive_controller.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])



    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # delayed_diff_drive_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[diff_drive_controller_spawner],
    #     )
    # )

    

    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )


    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )



     # Create the launch description and populate
    ld = LaunchDescription()
    

    # Add the nodes to the launch description
    ld.add_action(rsp)
    # ld.add_action(rviz_node)
    # ld.add_action(joint_state_publisher_node)

    if use_ros2_control=='true':
        ld.add_action(delayed_controller_manager)
        ld.add_action(delay_joint_state_broadcaster_spawner)
        ld.add_action(delay_rviz_after_diff_drive_controller_spawner)
        ld.add_action(delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner)

    
    return ld      # return (i.e send) the launch description for excecution

